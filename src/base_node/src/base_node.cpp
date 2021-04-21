#include <ros/ros.h>
#include <base_node/dealLampCmd.h>
#include <base_node/base.h>

using std::string;

serial::Serial *baseRobotSPort;              //Serial对象
//serial::Serial *baseLampSPort;              //Serial对象

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "base_node");
    ros::NodeHandle nh;  
    

    string serial_stm1,serial_stm2;
    int baud_stm1,timeout1,baud_stm2,timeout2;
   
    nh.param<std::string>("/serial1_cfg/SERIAL_STM",serial_stm1,"ttyUSB0");
    nh.param("/serial1_cfg/BAUD_STM",baud_stm1,9600);
    nh.param("/serial1_cfg/TIMEOUT",timeout1,1000);

    nh.param<std::string>("/serial2_cfg/SERIAL_STM",serial_stm2,"ttyS1");
    nh.param("/serial2_cfg/BAUD_STM",baud_stm2,9600);
    nh.param("/serial2_cfg/TIMEOUT",timeout2,1000);

    serial::Timeout timeout1_ = serial::Timeout::simpleTimeout(timeout1);   //串口通信超时时间
    //serial::Timeout timeout2_ = serial::Timeout::simpleTimeout(timeout2);

    //与底盘STM32通信串口(底盘相关)
    baseRobotSPort = new serial::Serial(serial_stm1, baud_stm1,timeout1_);  

    //与扩展STM32通信串口(消毒灯相关)
    //baseLampSPort = new serial::Serial(serial_stm2, baud_stm2,timeout2_); 


    Base m_base(baseRobotSPort);     //
    //DealLampCmd dealLampCmd(baseLampSPort); //start a server，wait for client cmd.
   
    ros::MultiThreadedSpinner spinner(1);  //server执行时间较长，为server的回调函数打开一个 ros::spin()，防止阻塞其他回调
    ros::spin(spinner);
    return 0;
}

    /*
    try{baseRobotSPort->open();}
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    if(baseRobotSPort->isOpen())
        ROS_INFO("Serial is opened.");
    else 
        return -1;
    */

    /*
    try{ baseLampSPort->open();}
    catch(serial::IOException& e){
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }
    if( baseLampSPort->isOpen())
        ROS_INFO("Serial is opened.");
    else 
        return -1;
    */