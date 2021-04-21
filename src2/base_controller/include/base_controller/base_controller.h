#ifndef __BASE_NODE_H__
#define __BASE_NODE_H__
#include <string>
#include <ros/ros.h>
#include "serial/serial.h"
//#include <dg_console/DGConsole.h>
#include <std_msgs/String.h>
#include <boost/thread/thread.hpp>
//#include <dg_msgs/BaseInfo.h>
//#include <base/com.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
//#include <dg_msgs/wave.h>
//#include "dg_log/logger.h"
//#include <dg_msgs/NodeStatus.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>
#define CMD_LEN 32
#define PI 3.14159

#define LED_BIG_ON 0X01
#define LED_BIG_OFF 0XFE
#define LED_SMALL_ON 0X02
#define LED_SMALL_OFF 0X0D
#define RELAY_ON 0X04
#define RELAY_OFF 0X0B

#define COMMUNICATION_ERROR 0X01
#define CLEAR_COMMUNICATION_ERROR 0XFE
#define VOLTAGE_ERROR 0X02
#define CLEAR_VOLTAGE_ERROR 0X0D
#define PANTILT_ERROR 0X04
#define CLEAR_PANTILT_ERROR 0X0B
#define LASER_ERROR 0X08
#define CLEAR_LASER_ERROR  0X07
#define FLIR_ERROR 0X10
#define CLEAR_FLIR_ERROR 0XEF
#define HIK_ERROR 0X20
#define CLEAR_HIK_ERROR 0XDF
#define MICROPHONE_ERROR 0X40
#define CLEAR_MICROPHONE_ERROR 0XBF

#define ACK_SLEEP_CMD 0x01
#define ACK_WAKEUP_CMD 0x02

typedef short int16;
typedef unsigned short uint16;
typedef int int32;
typedef unsigned int uint32;
typedef char int8;
typedef unsigned char uint8;
typedef unsigned char byte;
typedef unsigned char uchar;

#define ByteCast
class base
{
public:

    base();
    ~base();

private:
    ros::NodeHandle nh;
    ros::NodeHandle private_nh;

    ros::Subscriber cmdSub;
    ros::Subscriber userSub;
    ros::Subscriber imuSub;
    ros::Subscriber robotPoseSub;
    void onCmd(const std_msgs::StringConstPtr &cmd);
    void onCallBackUser(const std_msgs::StringConstPtr &cmd);
    void onUpdateRobotPose(const geometry_msgs::PointStampedConstPtr&);
    void handleImuData(const sensor_msgs::Imu::ConstPtr&);
    ros::Subscriber TwistSub;
    void callBackCmdVel(const geometry_msgs::TwistConstPtr & vel);

    ros::Publisher baseInfoPub;
    ros::Publisher batteryInfoPub;
    ros::Publisher odomPub; 
    ros::Publisher audioPub;
    ros::Publisher statusPub;
    bool bBrakeon;
    //ros::Publisher wavePub;
    //void publishwave(const dg_msgs::wave &data);
    ros::Publisher naviCmdPub;
    ros::Publisher socketCmdPub;
	ros::Publisher baseHeartPub;
	ros::Publisher baseUserPub;
    ros::ServiceClient sleepstateCall;
    float robot_poseR_;
    tf::TransformBroadcaster tfBroadcaster;
    boost::thread* baseThread;
    boost::thread* batteryThread;
    boost::thread* sonarThread;
    boost::shared_mutex imu_mx_;
    boost::shared_mutex robot_pos_mx_;
    //CCom baseCom;
    //CCom batteryCom;
    //CCom sonarCom;
	serial::Serial *baseCom;
	serial::Serial *baseUser;
    bool auto_awake_laser_;
    std::string baseComName;
    std::string baseUserName;
    std::string sonarComName;
    double imu_Velz;
    double imu_w;
    bool b_imu_offset;
    bool b_imu_first_data_;
    int loopRate;
    double initR;
	bool b_auto_sleep;
	bool b_modify_odom_;
    bool b_use_imu_;
    double maxLineSpeed;
    double maxTurnSpeed;

    bool useSpeedSmooth;
    bool useJoyStickSpdTest;
    double speedStepBig;
    double speedStepSmall;
    int spdKeepTimeJoystick;
    

    std::string comFile;
    std_msgs::String str;
    int detect_count;
    bool use_sonar;
    bool new_battery;

    float poseX;
    float poseY;
    float poseR;
    float velocityX;
    float velocityY;
    float velocityR;
    float velX;
    float velY;
    float velR;

    //monitor thread state
    bool batt_thread_flag_;
    bool baseLoop_thread_flag_;

#define TEST_ODOM_CHECK
#ifdef TEST_ODOM_CHECK   
    bool printOdomFromCtrlBrd;
    float testOdomAxisX;    
    float testOdomAxisY;    
    float testOdomDegreeR;
#endif
    float imu_offset;


    int zeroSpdRpeatTimes;
	int userCmd;
#define IN_SLEEP 1
#define NOT_SLEEP 0
#define QUIT_SLEEP 2

    bool m_boardAckBits[8];
    bool sleepState;


private:
    void baseLoop();
	void batteryLoop();
    void processStatusInfo(byte statusHighByte, byte statusLowByte);
    void odom_pub();

    void processCmd(const char * cmd);
    void Move(float x, float y, float r, bool delay = false);
    void Speed(float x, float y, float r, bool delay = false);

    size_t GetControlCmd(int8 moveType, int8 vel_1, int8 vel_2, int8 vel_3, byte *cmd);
    size_t GetQueryCmd(byte* cmd);

    byte  CRC8( const byte *buf, byte len);
    byte  CheckCRC(byte *crctmp, byte len);
    void  ProcessACK(byte ack);
};

#endif
