#include "base_node/dealLampCmd.h"

DealLampCmd::DealLampCmd(serial::Serial* m_serialPort):serialPort(m_serialPort)
{  
    n.param("/serial2_cfg/RATE",rate,10);

    ctrlLampServer_ = new CtrlLampServer(n,"ctrl_lamp",boost::bind(&DealLampCmd::dealCmd,this,_1),false);
    //last param decide whether to auto_start server
    ctrlLampServer_->start();
    ctrlLampServer_->registerPreemptCallback(boost::bind(&DealLampCmd::preemptCb,this));
    ROS_INFO("---start ctrl_lamp server---"); 
}

DealLampCmd::~DealLampCmd()
{
    if(ctrlLampServer_ != NULL)
    {
        delete ctrlLampServer_;
        ctrlLampServer_ = NULL;
    }
}

int DealLampCmd::transCmd(const char* cmd)
{
    int userCmd;
    if(strcmp(cmd, "dev:up") == 0 )
	{
    	ROS_INFO("onCallBackUser:1");
		userCmd = 1;
	}
	else if(strcmp(cmd, "dev:down") == 0 )
	{
    	ROS_INFO("onCallBackUser:2");
		userCmd = 2;
	}
	else if( strcmp(cmd, "dev:all") == 0 )
	{
    	ROS_INFO("onCallBackUser:3");
		userCmd = 3;
	}
	else if( strcmp(cmd, "dev:check") == 0 )
	{
    	ROS_INFO("onCallBackUser:0");
		userCmd = 0;
	}
    return userCmd;
}

void DealLampCmd::dealCmd(const base_node::CtrlLampGoalConstPtr& goal)
{
    int userCmd = -1;
    base_node::CtrlLampFeedback feedback;
    userCmd = transCmd(goal->task.c_str());
    if(userCmd != -1 )
    {
        unsigned char cmd[12]={0XFE,0XEE,0X05,0X01,0X00,0X00,0X00,0X00,0X55,0X55,0XFC,0XFF};
        cmd[3] = userCmd;
        unsigned char rcv[1024];
        
        ROS_INFO("deal userCmd");

        int sizeRcv;
        serialPort->waitByteTimes(100);
        ros::Rate loop_rate(rate);
        preemptIsRun = false;
        //向stm32写指令cmd
        if(serialPort->write(cmd, sizeof(cmd)) == sizeof(cmd))
        {
            if(serialPort->waitReadable())
            {
                sizeRcv = serialPort->read(rcv,sizeof(rcv));
                if(sizeRcv < 12 ) ROS_INFO("error rcv");
                else
                {
                    if(( rcv[0] == 0xfe )&&( rcv[1] == 0xee )&&( rcv[10] == 0xfc )&&( rcv[11] == 0xff ))
                    {
                        ROS_INFO("deal userCmd ok");
                    }
                }
            }
        }
        //发送check指令检测状态
        userCmd = transCmd("dev:check");
        cmd[3] = userCmd;
        userCmd = -1;
        while(ros::ok())
        {
            if(serialPort->write(cmd, sizeof(cmd)) == sizeof(cmd))
            {
                if(serialPort->waitReadable())
                {
                    //ROS_INFO("true");
                    sizeRcv = serialPort->read(rcv,sizeof(rcv));
                    if(sizeRcv < 12 ) ROS_INFO("error rcv");
                    else
                    {
                        if(( rcv[0] == 0xfe )&&( rcv[1] == 0xee )&&( rcv[10] == 0xfc )&&( rcv[11] == 0xff ))
                        {
                            feedback.taskProgress = rcv[7];
                            ctrlLampServer_->publishFeedback(feedback);
                            //printf("%x",feedback.taskProgress); ////////////////////////////第一次的状态 继续down
                            if(rcv[7]== 0Xff) break;  //task is finnished
                        }
                    }
                }
            }
            loop_rate.sleep();
        }  //while(ros::ok())
        base_node::CtrlLampResult result;
        result.taskIsFininshed = true;
        if(!preemptIsRun)  ctrlLampServer_->setSucceeded(result);  
    }
}

//构造函数中注册了当客户端发出cancelGoal()时的响应函数
void DealLampCmd::preemptCb()
{
    preemptIsRun = true;
    ctrlLampServer_->setPreempted();//强制中断
}

