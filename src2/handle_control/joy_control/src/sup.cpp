//#include <iostream>
#include <strstream>
#include <ros/ros.h>
#include "sup.h"
#include "dg_log/logger.h"
#include <unistd.h>

supTele::supTele():
     DGConsole("sup",boost::bind(&supTele::processCmd, this, _1))
{
    sub = nh.subscribe<std_msgs::String>("sup_cmd", 10, &supTele::OnCmd, this);
    pub = nh.advertise<std_msgs::String>("sup_info", 10);
}

void supTele::processCmd(const char *cmd)
{
    std_msgs::String msg;
    msg.data = string(cmd);
    pub.publish(msg);
}

void supTele::OnCmd(const std_msgs::StringConstPtr &msg)
{
    int LOG_BUF_lEN = 32;
    char buf[LOG_BUF_lEN];
    ostrstream strout(buf, LOG_BUF_lEN);

    strout << "supTele::OnCmd, msg->data: " << "[" << msg->data << "]";
    
    strout << ends; //'\0'
    buf[LOG_BUF_lEN - 1] = '\0'; //防止缓冲区满，没有字符终结符，%s打印会越界
    //ROS_INFO("%s\n", buf);
	LOGS_INFO("joy_teleop")<<":"<<buf;
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "sup");
	LOGS_SET_SWITCH_TIME_MS("joy_teleop",1000*86400);
    supTele cl;
    ros::spin();

    return 1;
}
