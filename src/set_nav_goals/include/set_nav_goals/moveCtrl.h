#ifndef MOVVE_CTRL_H_
#define MOVVE_CTRL_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "netbase_msgs/netbase_msgs.h"
#include <ros/callback_queue.h>
#include <boost/thread/thread.hpp>
#include "zjucmd.h"

/*控制底盘移动，M1前进，M2后退，M3左转，M4右转*/
class MoveCtrl
{
public:
    MoveCtrl();
    ~MoveCtrl();
    bool code_seen(char code);
    unsigned int code_value(void);
    bool VelSmooth();
    void packetCallback(const netbase_msgs::netbase_msgs::ConstPtr &pack);
    void MoveCtrlInfoLoop();
private:
    double target_velx;
    double target_velyaw;    
    double cur_velx ;
    double cur_velyaw ;
    geometry_msgs::Twist cmd_vel;
    
    char *cmdbuffer;
    int bufindr;
    char *strchr_pointer;

    ros::Publisher vel_pub;
    ros::NodeHandle nh;
    ros::Subscriber packet_sub;

    boost::thread* MoveCtrlInfoThread;
    ros::CallbackQueue packetCallbackQueue;
    ros::SubscribeOptions packetSubOptions;

    int rate;
};


#endif