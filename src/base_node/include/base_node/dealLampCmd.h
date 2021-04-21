#ifndef DEAL_LAMP_CMD_H_
#define DEAL_LAMP_CMD_H_

#include <actionlib/server/simple_action_server.h>
#include "base_node/CtrlLampAction.h"
#include <ros/ros.h>
#include <serial/serial.h>

typedef actionlib::SimpleActionServer<base_node::CtrlLampAction> CtrlLampServer;

class DealLampCmd
{
public:
    DealLampCmd(serial::Serial* m_serialPort);
    virtual ~DealLampCmd();
    void dealCmd(const base_node::CtrlLampGoalConstPtr& goal);
    int transCmd(const char* cmd);
    void dealCmd1();
    

private:
    /* data */
    CtrlLampServer* ctrlLampServer_;
    ros::NodeHandle n;
    serial::Serial *serialPort;
    bool preemptIsRun;
    int rate;
    void preemptCb();
};



#endif