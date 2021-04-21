#include<base_controller/base_controller.h>
#include <stdio.h>
#include <ros/ros.h>
//#include "dg_log/logger.h"
#include <unistd.h>
//#include "logbasedef.hpp"


int main(int argc, char** argv)
{
    char ver_[255];
    memset(ver_,0,255);
    sprintf(ver_,"Software Compiled Time: %s, %s. ",__DATE__, __TIME__);

    ros::init(argc, argv, "base_node");
    //LOGS_SET_SWITCH_TIME_MS("base_node",1000*86400);
    base m_base;
    //LOGS_INFO("base_node")<<"****************";
    //LOGS_INFO("base_node")<<"base_node "<<ver_;
    //LOGS_INFO("base_node")<<"****************";

    ros::spin();
}
