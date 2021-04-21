/*控制底盘移动，M1前进，M2后退，M3左转，M4右转*/

#include "set_nav_goals/moveCtrl.h"
#include <unistd.h>
    
MoveCtrl::MoveCtrl()
{
    rate = 20;
    cur_velx = 0;
    cur_velyaw = 0;
    bufindr = 0;

    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    packetSubOptions = ros::SubscribeOptions::create<netbase_msgs::netbase_msgs>(
        "netbase_user2server",100,boost::bind(&MoveCtrl::packetCallback, this,_1),ros::VoidPtr(),&packetCallbackQueue);
    packet_sub = nh.subscribe(packetSubOptions);

    MoveCtrlInfoThread = new boost::thread(boost::bind(&MoveCtrl::MoveCtrlInfoLoop, this)); //为上位机软件遥控底盘运动单独开一个线程
}

MoveCtrl::~MoveCtrl()
{

}


void MoveCtrl::packetCallback(const netbase_msgs::netbase_msgs::ConstPtr &pack)
{
    cmdbuffer = (char *)pack->data.c_str();
    //char * ext = process_commands_ext(cmdbuffer);
	if (code_seen('M') == true)
	{
        if (code_value() == 1)
		{
            target_velx = 0.4;
		}
		else if (code_value() == 2)
		{
            target_velx = -0.4;
		}
		else if (code_value() == 3)
		{
            target_velyaw = 0.5;
		}
	    else if (code_value() == 4)
		{
            target_velyaw = -0.5;
		}
        else  
        {
            for (int i=10; i>0; i--)
            {
                target_velx = 0;
                target_velyaw = 0;
            }
        }
        //VelSmooth();
	}
}

bool MoveCtrl::VelSmooth()
{
    double eps = 0.0000001;
    int speedCount = 20;
    double speedStepx = (target_velx - cur_velx)/speedCount;
    double speedStepYaw = (target_velyaw - cur_velyaw)/speedCount;
    /*speed up*/
    if (fabs(target_velx)>eps || (fabs(target_velyaw)>eps))
    {
        if (fabs(target_velx)>eps)
        {
            cur_velx = fabs(target_velx - cur_velx) > eps ? (cur_velx + speedStepx) : target_velx;          
        }
        if (fabs(target_velyaw)>eps)
        {
            cur_velyaw = fabs(target_velyaw - cur_velyaw) > eps ? (cur_velyaw + speedStepYaw) : target_velyaw;
        }
        cmd_vel.linear.x = cur_velx;
        cmd_vel.angular.z = cur_velyaw;
        vel_pub.publish(cmd_vel);          
    }
    /*speed down*/
    else
    {
        for(int i=0; i<speedCount; i++)
        {
            cur_velx = fabs(target_velx - cur_velx) > 0.02 ? cur_velx + speedStepx : target_velx;
            //cur_velyaw = fabs(target_velyaw - cur_velyaw) > 0.05 ? cur_velyaw + speedStepYaw: target_velyaw;
            cur_velyaw = 0; //旋转行为可以不设置平滑，抖动不大
            cmd_vel.linear.x = cur_velx;
            cmd_vel.angular.z = cur_velyaw;
            vel_pub.publish(cmd_vel);
            usleep(50000); //形参为整形，单位为微秒  ，此处延时60000微秒，即0.06s      
        }
        cmd_vel.linear.x = 0;
        cmd_vel.angular.z = 0;
        vel_pub.publish(cmd_vel);       
    }
    return true;
}

void MoveCtrl::MoveCtrlInfoLoop()
{
    ros::Rate loop_rate(rate);
    while (ros::ok())
    {
        packetCallbackQueue.callOne(ros::WallDuration(0));
        loop_rate.sleep(); 
    }
}

bool MoveCtrl::code_seen(char code)
{
    strchr_pointer = strchr(&cmdbuffer[bufindr], code);
    return (strchr_pointer != NULL);   
}

unsigned int MoveCtrl::code_value(void)
{
    return (strtod(&cmdbuffer[strchr_pointer -cmdbuffer + 1], NULL));
}



