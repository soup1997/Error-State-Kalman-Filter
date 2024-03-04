#include <eskf/eskf.hpp>

int main(int argc, char** argv){

    ros::init(argc, argv, "ESKF");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~"); // using private parameters in node
    ESKF eskf(nh, private_nh);

    while(ros::ok()) ros::spinOnce();

    return 0;
}