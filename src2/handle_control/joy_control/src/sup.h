#include <iostream>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <dg_console/DGConsole.h>

using namespace  std;


class supTele: public DGConsole
{
public:
    supTele();

    void processCmd(const char * cmd);

    ros::NodeHandle nh;
    ros::Subscriber sub;
    void OnCmd(const std_msgs::StringConstPtr &msg);
    ros::Publisher pub;
};
