#include <eskf/eskf.hpp>

ESKF::ESKF(ros::NodeHandle nh, ros::NodeHandle private_nh): save(true)
{
    /*------------Load Predefined Prameters in YAML-----------*/
    private_nh.param<std::string>("imu_topic", imu_topic, "kitti/oxts/imu");
    ROS_INFO("Imu topic: %s", imu_topic.c_str());

    private_nh.param<std::string>("vo_topic", vo_topic, "kitti/vo");
    ROS_INFO("VO topic: %s", vo_topic.c_str());

    private_nh.param<std::string>("sp_vo_topic", sp_vo_topic, "kitti/sp_vo");
    ROS_INFO("SP VO topic: %s", sp_vo_topic.c_str());

    private_nh.param<std::string>("sp_vio_topic", sp_vio_topic, "kitti/sp_vio");
    ROS_INFO("SP VIO topic: %s", sp_vio_topic.c_str());

    /*---------Pose Variables----------*/
    tf << -0.0010886, -0.0085120,  0.9999632,
          -0.9999764, -0.0067777, -0.0011463,
           0.0067872, -0.9999408, -0.0085044;

    /*------------Nominal State Variables-----------*/

    pos = Eigen::Vector3f::Zero();
    vel = Eigen::Vector3f::Zero();
    quat = Eigen::Quaternionf::Identity();
    
    gravity << 0.0, 0.0, -GRAVITY;
    R = quat.matrix();

    /*------------Error State Variables-----------*/
    dpos.setZero();
    dvel.setZero();
    dtheta.setZero();

    P.setIdentity(); // [9 X 9] matrix
    P *= 1e-5;

    Fx.setZero(); // [9 X 9] matrix

    Fi.setZero();              // [9 X 6] matrix
    Fi.block<3, 3>(3, 0) = I3; // velocity covariance
    Fi.block<3, 3>(6, 3) = I3; // angle covariance

    V.setIdentity(); // (tx, ty, tz, qw, qx, qy, qz)
    V.block<3, 3>(0, 0).diagonal() << 1e-3, 1e-3, 1e-3;
    //V.block<4, 4>(3, 3).diagonal() << 2e-4, 2e-4, 2e-4, 2e-4;

    I.setIdentity();

    G.setZero();

    /*------------ROS Pub-Sub Definition-----------*/
    imu_sub = nh.subscribe(imu_topic, 1, &ESKF::imuCallback, this);
    vo_sub = nh.subscribe(vo_topic, 1, &ESKF::voCallback, this);
    sp_vo_pub = nh.advertise<nav_msgs::Path>(sp_vo_topic, 1);
    sp_vio_pub = nh.advertise<nav_msgs::Path>(sp_vio_topic, 1);

    if (save){
        fout.open("/home/smeet/catkin_ws/src/Visual-Inertial-Odometry/eskf/results/10_superpoint_vio.txt");
        fout << std::fixed << std::setprecision(6);
    }
}

void ESKF::pathPublisher(void)
{
    static nav_msgs::Path path;
    static int seq = 0;

    geometry_msgs::PoseStamped pose;

    path.header.frame_id = "world";
    path.header.seq = seq;
    path.header.stamp = ros::Time::now();

        
    pose.header.frame_id = path.header.frame_id;
    pose.header.seq = path.header.seq;
    pose.header.stamp = path.header.stamp;


    pose.pose.position.x = pos(0);
    pose.pose.position.y = pos(1);
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = quat.x();
    pose.pose.orientation.y = quat.y();
    pose.pose.orientation.z = quat.z();
    pose.pose.orientation.w = quat.w();

    path.poses.push_back(pose);
    sp_vio_pub.publish(path);

    seq++;
}

void ESKF::superPointPublisher(Eigen::Vector3f &meas_pos, Eigen::Quaternionf &meas_quat)
{
    static nav_msgs::Path path;
    static int seq = 0;

    geometry_msgs::PoseStamped pose;

    path.header.frame_id = "world";
    path.header.seq = seq;
    path.header.stamp = ros::Time::now();

        
    pose.header.frame_id = path.header.frame_id;
    pose.header.seq = path.header.seq;
    pose.header.stamp = path.header.stamp;


    pose.pose.position.x = meas_pos(0);
    pose.pose.position.y = meas_pos(1);
    pose.pose.position.z = 0.0;

    pose.pose.orientation.x = meas_quat.x();
    pose.pose.orientation.y = meas_quat.y();
    pose.pose.orientation.z = meas_quat.z();
    pose.pose.orientation.w = meas_quat.w();

    path.poses.push_back(pose);
    sp_vo_pub.publish(path);

    seq++;
}

void ESKF::imuCallback(const sensor_msgs::Imu::ConstPtr &msg)
{
    static std::chrono::system_clock::time_point prev_time(std::chrono::system_clock::now());

    if (msg == nullptr)
    {
        ROS_WARN("Received an empty imu message...");
        return;
    }

    else
    {
        std::chrono::system_clock::time_point start(std::chrono::system_clock::now());
        {   
            /*------------imuCallback Main-----------*/
            std::unique_lock<std::mutex> lock(mu);
            std::chrono::system_clock::time_point curr_time(std::chrono::system_clock::now());
            std::chrono::duration<float> duration(curr_time - prev_time);

            float dt = duration.count();

            Eigen::Vector3f a_m(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
            Eigen::Vector3f w_m(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z);
            Eigen::Quaternionf q_m(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);

            pred_nominal_state(a_m, w_m, q_m, dt);
            pred_error_state(a_m, w_m, q_m, dt);
            pred_covariance(dt);

            prev_time = curr_time;
        }
        std::chrono::system_clock::time_point end(std::chrono::system_clock::now());
        std::chrono::milliseconds elapsed(std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
        std::cout << "Execution time of imuCallback: " << elapsed.count() << " ms\n";
        // pathPublisher();
    }
}

void ESKF::voCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    if (msg == nullptr)
    {
        ROS_WARN("Received an empty visual odometry message...");
        return;
    }

    else
    {
        {
            /*------------voCallback Main-----------*/
            std::unique_lock<std::mutex> lock(mu);

            float tx = msg->pose.position.x;
            float ty = msg->pose.position.y;
            float tz = msg->pose.position.z;
            
            float qw = msg->pose.orientation.w;
            float qx = msg->pose.orientation.x;
            float qy = msg->pose.orientation.y;
            float qz = msg->pose.orientation.z;
            
            Eigen::Quaternionf meas_quat(qw, qx, qy, qz);
            Eigen::Vector3f meas_translation(tx, ty, tz);

            Eigen::Vector3f model_pos = tf * meas_translation;
            Eigen::Quaternionf model_quat(tf * meas_quat.matrix());
            
            correction(model_pos, model_quat);
            superPointPublisher(model_pos, model_quat);
            pathPublisher();

            Eigen::Vector3f output_pos = tf.transpose() * pos;
            Eigen::Quaternionf output_quat(tf.transpose() * quat.matrix());

            Eigen::Matrix4f output = toSE3(output_quat, output_pos);
            
            fout << output(0,0) << " " << output(0,1) << " " << output(0,2) << " " << output(0,3) << " "
                 << output(1,0) << " " << output(1,1) << " " << output(1,2) << " " << output(1,3) << " "
                 << output(2,0) << " " << output(2,1) << " " << output(2,2) << " " << output(2,3) << "\n";
        }
    }
}

void ESKF::pred_nominal_state(Eigen::Vector3f &a_m, Eigen::Vector3f &w_m, Eigen::Quaternionf &q_m, float &dt)
{
    Eigen::Vector3f position = (vel * dt) + (0.5f * ((R * a_m) + gravity) * SQ(dt));
    Eigen::Vector3f velocity = ((R * a_m) + gravity) * dt;
    Eigen::Vector3f theta = w_m * dt;

    pos += position;
    vel += velocity;
    // quat = quat * toQuat(theta);
    quat = q_m;
    // R = quat.matrix();
}

void ESKF::pred_error_state(Eigen::Vector3f &a_m, Eigen::Vector3f &w_m, Eigen::Quaternionf &q_m, float &dt)
{
    Fx.setZero();

    Fx.block<3, 3>(0, 0) = I3;
    Fx.block<3, 3>(0, 3) = dt * I3;

    Fx.block<3, 3>(3, 3) = I3;
    Fx.block<3, 3>(3, 6) = -R * toSkew(a_m) * dt;
    // Fx.block<3, 3>(6, 6) = toQuat(w_m * dt).matrix().transpose();
    Fx.block<3, 3>(6, 6) = q_m.matrix().transpose();

    // dx = Fx * dx;
}

void ESKF::pred_covariance(float &dt)
{
    Eigen::Matrix<float, 6, 6> Q; // random impulse covariance
    Q.setZero();

    v_i = 4e-2 * SQ(dt) * I3;     // velocity random impulse
    theta_i = 3e-2 * SQ(dt) * I3; // angle random impulse

    Q.block<3, 3>(0, 0) = v_i;
    Q.block<3, 3>(3, 3) = theta_i;

    P = (Fx * P * Fx.transpose()) + (Fi * Q * Fi.transpose());
}

void ESKF::correction(Eigen::Vector3f &meas_pos, Eigen::Quaternionf &meas_quat)
{

    Eigen::Vector3f residual_pos = meas_pos - pos;
    Eigen::Quaternionf residual_quat = meas_quat * quat.conjugate();
    // Eigen::Vector3f residual_theta = 2.0f * residual_quat.vec() / residual_quat.w();

    Eigen::Matrix<float, MEASUREMENT_SIZE, ERROR_STATE> H;
    Eigen::Matrix<float, MEASUREMENT_SIZE, NOMINAL_STATE> Hx;
    Eigen::Matrix<float, NOMINAL_STATE, ERROR_STATE> Xdx;

    H.setZero();
    Hx.setZero();
    Xdx.setZero();

    Hx.block<3, 3>(0, 0) = I3;
    // Hx.block<4, 4>(3, 6) = I4;

    Xdx.block<3, 3>(0, 0) = I3;
    Xdx.block<3, 3>(3, 3) = I3;
    Xdx.block<4, 3>(6, 6) = getQdtheta(quat);

    H = Hx * Xdx;

    Eigen::Matrix<float, ERROR_STATE, MEASUREMENT_SIZE> K = (P * H.transpose()) * (H * P * H.transpose() + V).inverse();

    P = (I - K * H) * P;

    Eigen::VectorXf dx(ERROR_STATE);
    dx = K * residual_pos;

    dpos = dx.segment(0, 3);
    dvel = dx.segment(3, 3);
    dtheta = dx.segment(6, 3);

    pos += dpos;
    vel += dvel;
    quat = quat * toQuat(dtheta);
    R = quat.matrix();

    reset();
}

void ESKF::reset()
{
    Eigen::Matrix<float, ERROR_STATE, ERROR_STATE> G;
    G.setIdentity();

    G.block<3, 3>(6, 6) = I3 - (0.5 * toSkew(dtheta));
    P = G * P * G.transpose();

    dpos.setZero();
    dvel.setZero();
    dtheta.setZero();
}

ESKF::~ESKF(){
    fout.close();
}

/*
void VIO::imgCallback(const sensor_msgs::Image::ConstPtr &msg)
{
    static torch::TensorOptions options(torch::kFloat32);

    if (msg == nullptr)
    {
        ROS_WARN("Received an empty Image message...");
        return;
    }

    else
    {
        std::chrono::system_clock::time_point start(std::chrono::system_clock::now());

        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(*msg, sensor_msgs::image_encodings::BGR8);
        cv::Mat img = cv_ptr->image;
        cv::resize(img, img, cv::Size(640, 192));
        img.convertTo(img, CV_32FC3, 1.0f / 255.0f); // Convert to float and scale to [0, 1]
        torch::Tensor tensor_img = torch::from_blob(img.data, {1, 192, 640, 3}, options);
        tensor_img = tensor_img.permute({0, 3, 1, 2}); // [sequence, channel, height, width]
        tensor_imgs.push_back(tensor_img);

        if (tensor_imgs.size() >= sequence_length)
        {
            torch::Tensor input_tensor = torch::cat(tensor_imgs, 0).unsqueeze(0); // [1, 2, 3, 192, 640]
            input_tensor = input_tensor.to(torch::kCUDA);

            torch::Tensor output_tensor = trained_model.forward({input_tensor}).toTensor();
            output_tensor = output_tensor.to(torch::kCPU);
            std::cout << output_tensor << std::endl;

            std::vector<float> output_vec = Tensor2Vec(output_tensor);
            std::cout << output_vec << std::endl;


            if (outputFile.is_open())
            {
                for (const float &value : output_vec)
                {
                    outputFile << value << " ";
                }
                outputFile << std::endl; // Add a newline after each set of values
            }
            else
            {
                std::cerr << "Failed to open output_vec.txt for writing." << std::endl;
            }

            Eigen::Matrix4f rel_pose = ToSE3(output_vec);
            vehicle_pose *= rel_pose;
            tensor_imgs.erase(tensor_imgs.begin());

            std::chrono::system_clock::time_point end(std::chrono::system_clock::now());
            std::chrono::milliseconds elapsed(std::chrono::duration_cast<std::chrono::milliseconds>(end - start));
            std::cout << "Execution Time on imgCallback: " << elapsed.count() << " ms\n";

            pathPublisher(vehicle_pose);
        }
    }
}
*/