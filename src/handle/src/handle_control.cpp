#include<ros/ros.h>  
#include<geometry_msgs/Twist.h>  
#include <sensor_msgs/Joy.h>  
#include<iostream>  
using namespace std;  

static bool cnt_needSendZero=false;

class Teleop  
{  
public:  
    Teleop();  
    int judgeKey(const double & vel);
  
private:  
    /* data */  
    void callback(const sensor_msgs::Joy::ConstPtr& Joy);  
    ros::NodeHandle n; //实例化节点  
    ros::Subscriber sub ;  
    ros::Publisher pub ;  
    double vlinear,vangular;//我们控制乌龟的速度，是通过这两个变量调整  
    int key_walk;  //axes[]的键  
    int key_right,key_left;  //buttons[]的键 
};  
  
Teleop::Teleop()  
{     
    //下面按键的设置，一定要根据自己的实际情况来更改 
    n.param<int>("KEY_FORWARD_BACKWARD",key_walk,7); 
    n.param<int>("KEY_TURN_RIGHT",key_right,1);    
    n.param<int>("KEY_TURN_LEFT",key_left,2);

    n.param<double>("/walk_vel",vlinear,0.5); //默认axes[1]接收速度
    n.param<double>("/yaw_rate",vangular,0.5); //默认axes[1]接收速度   

    pub = n.advertise<geometry_msgs::Twist>("/cmd_vel",1);//将速度发给乌龟
    sub = n.subscribe<sensor_msgs::Joy>("joy",10,&Teleop::callback,this); //订阅游戏手柄发来的数据
}

int Teleop::judgeKey(const double & vel)
{
    if (vel < 0.0001 && vel > -0.0001)
        return 0;
    else if (vel >= 0.0001)
        return 1;
    else // vel < -0.001

        return -1;
}

void Teleop::callback(const sensor_msgs::Joy::ConstPtr& Joy)  
{   
    geometry_msgs::Twist v;
    v.linear.x = 0;
    v.angular.z = 0;
    bool needMove = false;
    int temp = judgeKey(Joy->axes[key_walk]);//5
    if (temp == 1)
    {
        needMove = true; 
        v.linear.x = vlinear; //将游戏手柄的数据乘以你想要的速度，然后发给乌龟
    }
    else if(temp == -1)
    {
        needMove = true; 
        v.linear.x = -vlinear; //将游戏手柄的数据乘以你想要的速度，然后发给乌龟
    }
    
    if(Joy->buttons[key_right]==1)
    {
        needMove = true; 
        v.angular.z =-vangular;
    }
    else if(Joy->buttons[key_left]==1)
    {
        needMove = true; 
        v.angular.z =vangular;
    }
        
    /*当按键按下去的时候才开始发布速度*/
    if(needMove)
    {
        pub.publish(v);
        cnt_needSendZero=true;
        ROS_INFO("linear:%f angular:%f",v.linear.x,v.angular.z); 
    }
    else
    {
        /*判断按键之前按过，发布停止速度，并将cnt_needSendZero置为false,这样停止速度不会一直发*/
        if(cnt_needSendZero)
        {
            cnt_needSendZero=false;
            for(int i=10;i>0;i--)
            {
                ROS_INFO("joy_teleop :stop move");
                /*回调函数开头部分将速度初始化为0*/
                pub.publish(v);
            }        
        }
    }
}  

int main(int argc,char** argv)  
{  
    ros::init(argc, argv, "logteleop");  
    Teleop telelog;
    ros::spin();
    return 0;  
}
