#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Header.h"
#include <time.h>
#include <sstream>
#include <tf/transform_broadcaster.h>
#include <std_msgs/String.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "batteryAndRangeSensor");
  ros::NodeHandle n;
  ros::Publisher  pub = n.advertise<std_msgs::String>("batteryAndRangeSensor", 10);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;
    char baseInfo[512];
    //R is Rangensensor, V and P is battery info
    char receive_data[30]= {0};
    receive_data[10] = 10;
    sprintf(baseInfo," R%d R%d R%d R%d R%d R%d R%d R%d V%d P%d",
                    receive_data[10],receive_data[11],receive_data[12],receive_data[13],
                    receive_data[14],receive_data[15],receive_data[16],receive_data[17],
                    receive_data[18],receive_data[19]);
    msg.data = baseInfo;

    pub.publish(msg);

    loop_rate.sleep();

  }
  return 0;
}