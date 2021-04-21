#ifndef BASE_H_
#define BASE_H_

#include <ros/ros.h>
#include <serial/serial.h>
#include <boost/thread/thread.hpp>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>		//用于接收CMD_VEL节点
#include <tf/transform_broadcaster.h>	//用于tf
#include <nav_msgs/Odometry.h>			//用于里程计
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <ros/callback_queue.h>
#include <std_msgs/Header.h>
#include "sensor_msgs/Range.h"

//通信协议部分参数
#define CMD_SPEEDSET 0x01        //速度控制指令
#define CMD_ODOMREQUEST 0x02        //请求下位机数据指令
#define PI 3.141593

class Base
{
public:
    Base(serial::Serial* m_serialPort);
    virtual ~Base();
    void baseInfoLoop();
    void velocityCallback(const geometry_msgs::Twist::ConstPtr &cmd_input);
    void pubOdomAndTF();
    void initOdomParam();
    void pubRangeSensor();
    void pubOneSensor(ros::Publisher &ultrasound_pub,  const char *topic, const char *tfName, uint8_t rangeData); 
    void pubBatteryAndRangeSensor();
private:
    /* data */
    ros::NodeHandle nh;
    ros::Publisher odom_pub;
    ros::Subscriber velocity_sub; 
    ros::Publisher baseInfo_pub;
    ros::Publisher ultrasound_pub1;
    ros::Publisher ultrasound_pub2;
    ros::Publisher ultrasound_pub3;
    ros::Publisher ultrasound_pub4;
    ros::Publisher ultrasound_pub5;
    ros::Publisher ultrasound_pub6;
    ros::Publisher ultrasound_pub7;
    ros::Publisher ultrasound_pub8;
    uint8_t receive_data[30];
    unsigned char send_buffer[11];
    short uint8_to_short( uint8_t *b );
    void odomRequest();
    serial::Serial* m_serialPort;
    boost::thread* baseInfoThread;
    int rate;
    double x_lim, y_lim, r_lim;
    ros::CallbackQueue velocityCallbackQueue;
    ros::SubscribeOptions velocitySubOptions;

    //publish odom and tf
    tf::TransformBroadcaster odom_bc;    //发布tf_tree
    double pos_temp[3];
    ros::Time now_stamp,last_stamp;             //时间帧，也用于计算速度位移
    ros::Duration dt_Duration;
    double dx,dy,dz;
    short x_sum_meas,y_sum_meas,r_sum_meas;
    nav_msgs::Odometry odom_inf;                //里程计信息
    geometry_msgs::TransformStamped odom_tf;    //tf转换信息
    geometry_msgs::Point odom_point;            //用于发布里程计位置信息
    geometry_msgs::Vector3 odom_point_tf;       //用于发送tf位移信息
    double orie;                                //航偏角
    geometry_msgs::Quaternion odom_quat;        //姿态信息（四元数）
    geometry_msgs::Vector3 vel_linear,vel_angular;      //线速度、角速度
    float covariance1[36];
    float covariance2[36];

};


#endif
