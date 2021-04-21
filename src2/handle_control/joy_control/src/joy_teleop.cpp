#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <fstream>
#include "dg_log/logger.h"
#include <unistd.h>
#include "dg_msgs/NodeStatus.h"

using namespace std;

class WeJoyTeleop
{
private:
    //运动
    int AXIS_FORWARD_BACKWARD;
    int AXIS_LEFT_RIGHT;
    int KEY_TURN_LEFT;
    int KEY_TURN_RIGHT;
    int KEY_SPEED_UP;
    //云台
    int KEY_PANTILT_ON;
    int KEY_PAN_LEFT;
    int KEY_PAN_RIGHT;
    int KEY_TILT_UP;
    int KEY_TILT_DOWN;
    int AXIS_EV_UP_DOWN;  

    //导航
    int KEY_PAUSE;
    int KEY_OBSTACLE;
    int KEY_POSE_WRITE;

    double walk_vel_;
    double run_vel_;
    double yaw_rate_;
    double yaw_rate_run_;
    double walk_vel_y_;
    double run_vel_y_;
    double pantilt_increment_;
    double ev_increment_;

    bool speedup_;
    bool navi_pause;
    bool navi_obstacle;
    int node_count;
    int node_start;
    int node_increment;
    int useTestBtn; //#手柄面板正面右侧4个按钮中的上下两个(Y A 或者 1 3)，用作测试用途时的分配 
                    //#0-不使用测试功能， 1-用作开关超声波避障，2-用作收缩充电杆
    
    std_msgs::String audiostr;
    geometry_msgs::Pose2D robotPose,robotPoseNew;
    geometry_msgs::Twist cmd_vel;
    ros::NodeHandle n;
    ros::Publisher pub;
    ros::Publisher cmdPub;
    ros::Publisher audioPub;
    ros::Publisher pantiltPub;
    ros::Publisher naviCmdPub;
    ros::Publisher baseCmdPub;  
    
public:
    WeJoyTeleop()
    {
        pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        cmdPub = n.advertise<std_msgs::String>("motor_manager_cmd", 2);
        audioPub = n.advertise<std_msgs::String>("audio_string", 1);
        pantiltPub = n.advertise<std_msgs::UInt16>("pantilt_control", 1);
        naviCmdPub = n.advertise<std_msgs::String>("navigation_cmd", 1);
        baseCmdPub = n.advertise<std_msgs::String>("base_cmd", 1);
        ros::NodeHandle n_private("~");
        //键码
        n_private.param("KEY_FORWARD_BACKWARD", AXIS_FORWARD_BACKWARD, 7);
        n_private.param("KEY_LEFT_RIGHT", AXIS_LEFT_RIGHT, 6);
        n_private.param("KEY_TURN_LEFT", KEY_TURN_LEFT, 2);
        n_private.param("KEY_TURN_RIGHT", KEY_TURN_RIGHT, 1);
        n_private.param("KEY_SPEED_UP", KEY_SPEED_UP, 2);
        n_private.param("KEY_PAUSE", KEY_PAUSE, 3);
        n_private.param("KEY_OBSTACLE", KEY_OBSTACLE, 4);
        n_private.param("KEY_POSE_WRITE", KEY_POSE_WRITE, 5);
        /*
        n_private.param("KEY_PANTILT_ON", KEY_PANTILT_ON, 6);
        n_private.param("KEY_PAN_LEFT", KEY_PAN_LEFT, 0);
        n_private.param("KEY_PAN_RIGHT", KEY_PAN_RIGHT, 2);
        n_private.param("KEY_TILT_UP", KEY_TILT_UP, 3);
        n_private.param("KEY_TILT_DOWN", KEY_TILT_DOWN, 1);
        n_private.param("KEY_EV_UP", AXIS_EV_UP_DOWN, 5);
        */
        //速度
        n_private.param("walk_vel", walk_vel_, 0.3);
        n_private.param("run_vel", run_vel_, 0.5);
        n_private.param("walk_vel_y", walk_vel_y_, 0.3);
        n_private.param("run_vel_y", run_vel_y_, 0.5);
        n_private.param("yaw_rate", yaw_rate_, 0.4);
        n_private.param("yaw_rate_run", yaw_rate_run_, 0.8);
        //导航
        n_private.param("node_start", node_start, 0);
        n_private.param("node_increment", node_increment, 2);
        n_private.param("pantilt_increment", pantilt_increment_, 5.0);
        n_private.param("ev_increment", ev_increment_, 0.03);

        n_private.param("useTestBtn", useTestBtn, 5); //手柄某些按钮作为测试功能时的功能选择
        
        ////////////////////////////////////////////////////////////////////////
        node_count = node_start;
        speedup_ = false;
        navi_pause = false;
        navi_obstacle=false;

    }
    
    //接收定位信息，用于按键8 自动生成路径点
    void PoseReceived(geometry_msgs::PointStampedConstPtr pose)
    {
        robotPoseNew.x = pose->point.x;
        robotPoseNew.y = pose->point.y;
        robotPoseNew.theta = pose->point.z;
        //ROS_INFO("Receive pose ok!");
    }
    
    void joyCmdSubCallback(const std_msgs::StringConstPtr &msg)
    {
        if(msg->data.find("exit") == 0)
        {
            ros::requestShutdown();
        }
    }
    
    //用于判断模拟量按键
    int judgeKey(const double & vel)
    {
        if (vel < 0.0001 && vel > -0.0001)
            return 0;
        else if (vel >= 0.0001)
            return 1;
        else // vel < -0.001
            return -1;
    }
    
    void joySubCallback(const sensor_msgs::Joy::ConstPtr &msg)
    {
      //  ROS_INFO("Receive KEY");
        //LOGS_INFO("joy_teleop")<<"Receive KEY";
       ros::Publisher statuPub   = n.advertise<dg_msgs::NodeStatus>("nodes_status", 1);
       dg_msgs::NodeStatus status_;
       status_.comment.data="";
       status_.statusCode=0;
       status_.nodeName.data="joy_control";
       statuPub.publish(status_);

        #if 1 //手柄面板正面右侧4个按钮中的上下两个(Y A 或者 1 3)，用作测试用途时的分配
        if(0 != useTestBtn) {
            std_msgs::String baseCmdMsg;
            if(1 == useTestBtn) {
                if (msg->buttons[3] == 1) {
                    baseCmdMsg.data = "sonaron";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"sonaron";
                }
                if (msg->buttons[0] == 1) {
                    baseCmdMsg.data = "sonaroff";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"sonaroff";
                }
            } else if(2 == useTestBtn) {
                if (msg->buttons[3] == 1) {
                    baseCmdMsg.data = "chargeon";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"chargeon";
                }
                if (msg->buttons[0] == 1) {
                    baseCmdMsg.data = "chargeoff";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"chargeoff";
                }
            } else if(3 == useTestBtn) {
                if (msg->buttons[3] == 1) {
                    baseCmdMsg.data = "falldetecton";
                    baseCmdPub.publish(baseCmdMsg);
                     LOGS_INFO("joy_teleop")<<"falldetecton";
                }
                if (msg->buttons[0] == 1) {
                    baseCmdMsg.data = "falldetectoff";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"falldetectoff";
                }
            } else if(4 == useTestBtn) {
                //同时控制超声波和激光
                if (msg->buttons[3] == 1) {
                    baseCmdMsg.data = "sonaron";
                    baseCmdPub.publish(baseCmdMsg);
                    baseCmdPub.publish(baseCmdMsg);
                    
                    baseCmdMsg.data = "falldetecton";
                    baseCmdPub.publish(baseCmdMsg);
                    baseCmdPub.publish(baseCmdMsg);
                    //LOGS_INFO("joy_teleop")<<"falldetect and sonaron";
                    LOGS_INFO("joy_teleop")<<"arm rise";
                }
                if (msg->buttons[0] == 1) {
                    baseCmdMsg.data = "sonaroff";
                    baseCmdPub.publish(baseCmdMsg);
                    baseCmdPub.publish(baseCmdMsg);
                    
                    baseCmdMsg.data = "falldetectoff";
                    baseCmdPub.publish(baseCmdMsg);
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"arm fall";
                }
            }
            else if(5 == useTestBtn){

                if (msg->buttons[3] == 1) {
                    baseCmdMsg.data = "sonaron";
                    baseCmdPub.publish(baseCmdMsg);
                    
                    baseCmdMsg.data = "falldetecton";
                    baseCmdPub.publish(baseCmdMsg);
                    //LOGS_INFO("joy_teleop")<<"falldetect and sonaron";
                    LOGS_INFO("joy_teleop")<<"arm rise";
                }
                if (msg->buttons[0] == 1) {
                      baseCmdMsg.data = "falldetectoff";
                    baseCmdPub.publish(baseCmdMsg);
                    
                    baseCmdMsg.data = "sonaroff";
                    baseCmdPub.publish(baseCmdMsg);

                    LOGS_INFO("joy_teleop")<<"arm fall";
                }

                if( std::abs(msg->axes[1])>0.0001 && msg->axes[1]>0){
                    baseCmdMsg.data = "sonaron";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"left arm rise";
                }else if(std::abs(msg->axes[1])>0.0001 &&  msg->axes[1]<0){
                    baseCmdMsg.data = "sonaroff";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"left arm fall";
                }

                if(std::abs(msg->axes[4])>0.0001 && msg->axes[4]>0) {
                     baseCmdMsg.data = "falldetecton";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"right arm rise";
                }else if(std::abs(msg->axes[4])>0.0001 &&  msg->axes[4]<0){
                    baseCmdMsg.data = "falldetectoff";
                    baseCmdPub.publish(baseCmdMsg);
                    LOGS_INFO("joy_teleop")<<"right arm fall";
                }

                if (msg->buttons[6] == 1 && msg->buttons[7] == 0) {
                    baseCmdMsg.data = "downpushrise";
                    baseCmdPub.publish(baseCmdMsg);

                    LOGS_INFO("joy_teleop")<<"down push rise";
                }else if (msg->buttons[6] == 0 && msg->buttons[7] == 1) {
                    baseCmdMsg.data = "downpushfall";
                    baseCmdPub.publish(baseCmdMsg);

                    LOGS_INFO("joy_teleop")<<"down push fall";
                }
            }

        }
        #endif
        
        if (msg->buttons[KEY_PAUSE] == 1) // 5
        {
            navi_pause = !navi_pause;
        //    ROS_INFO(navi_pause ? "navi paused by joystick" : "navi resume by joystick");
			LOGS_INFO("joy_teleop")<<"navi_pause ? navi paused by joystick : navi resume by joystick";
            std_msgs::String cmd;
            if(navi_pause)
            {  
                cmd.data = "pause";
                audiostr.data = "joy pause, joy pause";
            } 
            else
            {
                cmd.data = "resume";
                audiostr.data = "joy resume, joy resume";
            }
            naviCmdPub.publish(cmd);
            LOGS_INFO("joy_teleop")<<cmd.data.c_str();
            audioPub.publish(audiostr);
        }
        
        if (msg->buttons[KEY_OBSTACLE] == 1) // 4
        {
            navi_obstacle = !navi_obstacle;
        //    ROS_INFO(navi_obstacle ? "obstacle off" : "obstacle on");
			LOGS_INFO("joy_teleop")<<"navi_obstacle ? obstacle off : obstacle on";
            std_msgs::String cmd;
            if(navi_obstacle)
            {  
                cmd.data = "obstacle off";
                audiostr.data = "joy obstacle off, joy obstacle off";
            } 
            else
            {
                cmd.data = "obstacle on";
                audiostr.data = "joy obstacle on, joy obstacle on";
            }
             LOGS_INFO("joy_teleop")<<cmd.data.c_str();
            naviCmdPub.publish(cmd);
            audioPub.publish(audiostr);
        }
        if(msg->axes[KEY_POSE_WRITE] == -1) //采集导航点
        {
            robotPose = robotPoseNew;
            ofstream fp_nodes;
            fp_nodes.open("/home/robot/path/nodes.txt", ios::app | ios::out);
            if(!fp_nodes)
                return;
            fp_nodes << node_count << " " << 0 << " " \
                << setprecision(6) << robotPose.x << " " <<robotPose.y << " " <<robotPose.theta <<endl;
            fp_nodes.close();
            
            if(node_count >= 1) //达到两个点以上，开始连边
            {
                ofstream fp_edges;
                fp_edges.open("/home/robot/path/edges.txt", ios::app | ios::out);
                if(!fp_edges)
                    return;
                fp_edges << node_count - node_increment << " " << node_count << endl;
                fp_edges.close();
            }
            node_count += node_increment;
            audiostr.data = "Write pose ok!";
            audioPub.publish(audiostr);
          //  ROS_INFO("Write pose ok!");
			LOGS_INFO("joy_teleop")<<"Write pose ok!";
        }

        if (msg->axes[KEY_SPEED_UP] == -1)
        {
            speedup_ = true;
           // ROS_INFO("Speed up!");
			LOGS_INFO("joy_teleop")<<"Speed up!";
        }
        else
            speedup_ = false;

        memset((void*)&cmd_vel, 0, sizeof(geometry_msgs::Twist));
        bool needMove = false; //根据这个开关决定是否要发布速度?
        if (msg->buttons[KEY_TURN_LEFT] == 1) // 3
        {
            needMove = true;  
            
            if (speedup_)
                cmd_vel.angular.z += yaw_rate_run_;
            else
                cmd_vel.angular.z += yaw_rate_;
             LOGS_INFO("joy_teleop")<<"KEY_TURN_LEFT";
        }
        if (msg->buttons[KEY_TURN_RIGHT] == 1) // 1
        {
            needMove = true;
            if (speedup_)
                cmd_vel.angular.z -= yaw_rate_run_;
            else
                cmd_vel.angular.z -= yaw_rate_;
             LOGS_INFO("joy_teleop")<<"KEY_TURN_RIGHT";
        }  

        int temp = judgeKey(msg->axes[AXIS_FORWARD_BACKWARD]);//5

        if (temp == 1)
        {
            needMove = true; 
            
            if (speedup_)
                cmd_vel.linear.x += run_vel_;
            else
                cmd_vel.linear.x += walk_vel_;
             LOGS_INFO("joy_teleop")<<"AXIS_FORWARD_BACKWARD";
        }
        else if (temp == -1)
        {
            needMove = true;            

            if (speedup_)
                cmd_vel.linear.x -= run_vel_;
            else
                cmd_vel.linear.x -= walk_vel_;
            LOGS_INFO("joy_teleop")<<"AXIS_FORWARD_BEHIND";
        }

        
#if 0  //容易误操作(欲走直线时却左右位移)，屏蔽之
        //左右平移
        temp = judgeKey(msg->axes[AXIS_LEFT_RIGHT]);// 4
        if (temp == 1)
        {
            if (speedup_)
                cmd_vel.linear.y += run_vel_y_;
            else
                cmd_vel.linear.y += walk_vel_y_;
        }
        else if (temp == -1)
        {
            if (speedup_)
                cmd_vel.linear.y -= run_vel_y_;
            else
                cmd_vel.linear.y -= walk_vel_y_;
        }
#endif
        static bool cnt_needSendZero=false;
        if(needMove)
        {
            cnt_needSendZero=true;
        }
        if(needMove)//不能使用此判断，否则速度一直保持上次手柄按下时发布的速度，松开也不会停止，因为得不到发布速度0的机会
        {
            pub.publish(cmd_vel);
        }
        else
        {
	  if(cnt_needSendZero)
          {
            cnt_needSendZero=false;
            for(int i=10;i>0;i--)
           {
                 LOGS_INFO("joy_teleop")<<"stop move";
                 pub.publish(cmd_vel);
           }
           
          }
        }

    }
};
void sendHeartBeat()
{
   ros::NodeHandle nh;
    ros::Publisher statuPub   = nh.advertise<dg_msgs::NodeStatus>("nodes_status", 1);
    dg_msgs::NodeStatus status_;
    while(1)
      {
        status_.comment.data="";
        status_.statusCode=0;
        status_.nodeName.data="joy_control";
        statuPub.publish(status_);
        usleep(100000);
     }
}
int main(int argc, char** argv)
{
    ros::init(argc, argv, "joy_teleop");
	LOGS_SET_SWITCH_TIME_MS("joy_teleop",1000*86400);

    ros::NodeHandle n;
    WeJoyTeleop joy;
    boost::thread th(sendHeartBeat);
    th.detach();
    LOGS_INFO("joy_teleop")<<"";
    LOGS_INFO("joy_teleop")<<"****************";
    LOGS_INFO("joy_teleop")<<"--->Version:Shenhaoinfo20190424";
    LOGS_INFO("joy_teleop")<<"****************";

    ros::Subscriber robotPoseSub = n.subscribe("robot_pose", 100,&WeJoyTeleop::PoseReceived, &joy);
    ros::Subscriber sub = n.subscribe("joy", 1, &WeJoyTeleop::joySubCallback, &joy);
    ros::Subscriber joycmdSub = n.subscribe("joy_cmd", 1, &WeJoyTeleop::joyCmdSubCallback, &joy);
    ros::spin();
    return 0;
}
