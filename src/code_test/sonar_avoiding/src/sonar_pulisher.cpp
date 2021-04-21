#include "ros/ros.h"
#include "sensor_msgs/Range.h"
#include "std_msgs/Header.h"
#include <time.h>
#include <sstream>
#include <tf/transform_broadcaster.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talkerUltraSound");
  ros::NodeHandle n;
  ros::Publisher ultrasound_pub1 = n.advertise<sensor_msgs::Range>("sonar1", 10);
  ros::Publisher ultrasound_pub2 = n.advertise<sensor_msgs::Range>("sonar2", 10);
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok())
  {

    sensor_msgs::Range msg,msg2;
    std_msgs::Header header,header2;
    header.stamp = ros::Time::now();
    header.frame_id = "ultrasound";
    msg.header = header;
    msg.field_of_view = 20*3.14/180;
    msg.min_range = 0.2;
    msg.max_range = 5.0;
    msg.range = 1.0;//rand()%3;
    /*
    tf::TransformBroadcaster broadcaster;
      broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.1, 0.0, 0.2)),
        ros::Time::now(),"base_link", "ultrasound"));*/
    ultrasound_pub1.publish(msg);
    
    header2.stamp = ros::Time::now();
    header2.frame_id = "ultrasound2";
    msg2.header = header2;
    msg2.field_of_view = 20*3.14/180;
    msg2.min_range = 0.2;
    msg2.max_range = 5.0;
    msg2.range = 1.2;//rand()%3;

    ultrasound_pub2.publish(msg2);


    loop_rate.sleep();
    ++count;
  }
  return 0;
}