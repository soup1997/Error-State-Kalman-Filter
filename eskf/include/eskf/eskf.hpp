#ifndef ESKF_HPP
#define ESKF_HPP

#define POSITION_STATE 3
#define VELOCITY_STATE 3
#define QUATERNION_STATE 4
#define MEASUREMENT_SIZE 3

#define NOMINAL_STATE (POSITION_STATE + VELOCITY_STATE + QUATERNION_STATE)
#define ERROR_STATE (NOMINAL_STATE - 1)
#define GRAVITY 9.81

#define I3 Eigen::Matrix3f::Identity()
#define I4 Eigen::Matrix4f::Identity()
#define SQ(x) ((x) * (x))

#include <iostream>
#include <fstream>
#include <chrono>
#include <string>
#include <vector>
#include <mutex>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <eskf/util.hpp>

class ESKF
{
private:
    /*---------ROS Definition---------*/
    ros::Subscriber imu_sub; // Imu Subscriber
    ros::Subscriber vo_sub;  // VO Subscriber
    
    ros::Publisher sp_vo_pub; // vo publisher
    ros::Publisher sp_vio_pub; // vio publisher

    std::string imu_topic;
    std::string vo_topic;

    std::string sp_vo_topic;
    std::string sp_vio_topic;

    /*---------Pose Variables----------*/
    Eigen::Matrix3f tf;                 // transfrom camera pose to body pose

    /*------------Nominal State Variables-----------*/
    Eigen::Vector3f pos, vel;
    Eigen::Quaternionf quat;

    Eigen::Matrix3f R;       // from inertial to body frame
    Eigen::Vector3f gravity; // gravity vector

    /*------------Error State Variables-----------*/
    Eigen::Vector3f dpos;    // delta position
    Eigen::Vector3f dvel;    // delta velocity
    Eigen::Vector3f dtheta;  // delta orientation

    Eigen::Matrix3f v_i;     // velocity random impulse
    Eigen::Matrix3f theta_i; // angle random impulse

    /*------------State Propagation Variables-----------*/
    Eigen::Matrix<float, ERROR_STATE, ERROR_STATE> P; // error state covariance

    Eigen::Matrix<float, ERROR_STATE, ERROR_STATE> Fx; // error state transition matrix
    Eigen::Matrix<float, ERROR_STATE, 6> Fi;           // error state covariance transition matrix
    Eigen::Matrix<float, MEASUREMENT_SIZE, MEASUREMENT_SIZE> V;                      // covariance matrix of measurement
    Eigen::Matrix<float, ERROR_STATE, ERROR_STATE> I;

    Eigen::Matrix<float, ERROR_STATE, ERROR_STATE> G;

    std::mutex mu;
    bool save;
    std::ofstream fout; // for save result

public:
    ESKF(ros::NodeHandle nh, ros::NodeHandle private_nh);
    ~ESKF();

    void imuCallback(const sensor_msgs::Imu::ConstPtr &msg);
    void voCallback(const geometry_msgs::PoseStamped::ConstPtr &msg);
    
    void superPointPublisher(Eigen::Vector3f &meas_pos, Eigen::Quaternionf &meas_quat);
    void pathPublisher(void);

    void pred_nominal_state(Eigen::Vector3f &a_m, Eigen::Vector3f &w_m, Eigen::Quaternionf &q_m, float &dt);
    void pred_error_state(Eigen::Vector3f &a_m, Eigen::Vector3f &w_m, Eigen::Quaternionf &q_m, float &dt);
    void pred_covariance(float &dt);
    void correction(Eigen::Vector3f &meas_pos, Eigen::Quaternionf &meas_quat);
    void reset();
};

#endif