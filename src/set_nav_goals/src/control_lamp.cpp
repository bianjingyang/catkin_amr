
#include "set_nav_goals/control_lamp.h"

CtrlLamp::CtrlLamp():taskProgress(0x00),taskIsFininshed(false),setTaskEnable(true)
{
    ctrllampclient_ = new CtrlLampClient("ctrl_lamp",true);
    if (!ctrllampclient_->waitForServer(ros::Duration(10.0))) {
        ROS_INFO("---Waiting for the ctrl_lamp server to start---"); //后面的时间间隔ros::Duration(),设置了等待的时间,若超过该时间,则中断该节点
    }
    else{
        ROS_INFO("---Connected to ctrl_lamp server---"); 
    }
}

CtrlLamp::~CtrlLamp()
{
    if(ctrllampclient_ != NULL)
    {
        delete ctrllampclient_;
        ctrllampclient_ = NULL;
    }
}

void CtrlLamp::sendCmd(std::string task)
{
    set_nav_goals::CtrlLampGoal goal;
    goal.task = task;
    ctrllampclient_->sendGoal(goal, 
                       boost::bind(&CtrlLamp::doneCb,this,_1,_2), 
                       boost::bind(&CtrlLamp::activeCb,this),
                       boost::bind(&CtrlLamp::feedbackCb,this,_1));
    success_ = false;  
    taskIsFininshed = false;
}



void CtrlLamp::doneCb(const actionlib::SimpleClientGoalState& state,
            const set_nav_goals::CtrlLampResultConstPtr& result)
{
    if(state == state.SUCCEEDED){
        ROS_INFO("---Task State: SUCCEEDED---");
        success_ = true;  
        taskIsFininshed = result->taskIsFininshed;
    }
    else if(state == state.ABORTED)
    {
    }
    //ros::shutdown();  
}


void CtrlLamp::activeCb()
{
    ROS_INFO("Task Received!");
}


void CtrlLamp::feedbackCb(const set_nav_goals::CtrlLampFeedbackConstPtr& feedback)
{
    taskProgress = feedback->taskProgress;
}

void CtrlLamp::cancelGoal()
{
    ctrllampclient_->cancelGoal();
}






/*

// 当action完成后会调用该回调函数一次
void doneCb(const actionlib::SimpleClientGoalState& state,
        const learning_action::DoDishesResultConstPtr& result)
{
    ROS_INFO("Yay! The dishes are now clean");
    ROS_INFO("SSS %d",result->total_dishes_cleaned);
    if(state==state.SUCCEEDED){
        ROS_INFO("STATE");
    }
    ros::shutdown();
}

// 当action激活后会调用该回调函数一次
void activeCb()
{
    ROS_INFO("Goal just went active");
}

// 收到feedback后调用该回调函数
void feedbackCb(const learning_action::DoDishesFeedbackConstPtr& feedback)
{
    ROS_INFO(" percent_complete : %f ", feedback->percent_complete);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "do_dishes_client");

    // 定义一个客户端
    Client client("do_dishes", true);

    // 等待服务器端
    ROS_INFO("Waiting for action server to start.");
    client.waitForServer();
    ROS_INFO("Action server started, sending goal.");

    // 创建一个action的goal
    learning_action::DoDishesGoal goal;
    goal.dishwasher_id = 1;

    // 发送action的goal给服务器端，并且设置回调函数
    client.sendGoal(goal,  &doneCb, &activeCb, &feedbackCb);

    ros::spin();

    return 0;
}*/