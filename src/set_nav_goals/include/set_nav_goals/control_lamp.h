#ifndef CONTROL_LAMP_H_
#define CONTROL_LAMP_H_

#include <actionlib/client/simple_action_client.h>
#include "set_nav_goals/CtrlLampAction.h"
#include <std_msgs/String.h>

typedef actionlib::SimpleActionClient<set_nav_goals::CtrlLampAction> CtrlLampClient;


class CtrlLamp
{
public:
    CtrlLamp();
    virtual ~CtrlLamp();

    void doneCb(const actionlib::SimpleClientGoalState& state,
                const set_nav_goals::CtrlLampResultConstPtr& result);
    void activeCb();
	void feedbackCb(const set_nav_goals::CtrlLampFeedbackConstPtr& feedback);
    void sendCmd(std::string task);
    void cancelGoal();

    int taskProgress;
    bool taskIsFininshed;
    bool success_;
    bool setTaskEnable;

private:
    /* data */
    CtrlLampClient* ctrllampclient_;   
};

#endif