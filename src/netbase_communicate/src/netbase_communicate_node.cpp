#include <ros/ros.h>
#include <netbase_communicate/netbase_communicate.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "zjuagv_node");
    ros::NodeHandle node;
    ros::NodeHandle private_nh("~");

    if (!zjuagv_initialize()) {
        ROS_ERROR("Cannot initialize WIFI driver...");
        return 0;
    }
    while(ros::ok() && zjuagv_polling()) {
        ros::spinOnce();
    }
    return 0;
}
