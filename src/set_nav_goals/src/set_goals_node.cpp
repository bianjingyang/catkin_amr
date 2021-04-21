#include <set_nav_goals/set_goals.h>
#include "zjucode.h"
#include "zjucmd.h"
#include "zjudeal.h"
#include <netbase_msgs/netbase_msgs.h>
#include <std_msgs/UInt32.h>
#include <std_msgs/String.h>
#include "set_nav_goals/control_lamp.h"
#include "set_nav_goals/moveCtrl.h"

ros::Publisher packet_pub;
ros::Subscriber packet_sub; 
ros::Subscriber baseInfo_sub;

bool taskIsRunning = false;
bool isLastPos = false;

CtrlLamp *ctrlLamp;


/*subscribe packet from upper PC and send dealing results to uppper PC */
void packetCallback(const netbase_msgs::netbase_msgs::ConstPtr &pack)
{
    double time1 = ros::Time::now().toSec();	
	//deal CMD from upper PC
	char * ext = process_commands_ext((char *)pack->data.c_str());
	/**/
	if(strcmp(ext,"T1")==0)
	{
		ctrlLamp->cancelGoal();
        ctrlLamp->sendCmd("dev:up");
	}
	else if(strcmp(ext,"T2")==0)
	{
		ctrlLamp->cancelGoal();
        ctrlLamp->sendCmd("dev:down");
	}
	else if(strcmp(ext,"T0")==0)
	{
		ctrlLamp->cancelGoal();
        ctrlLamp->sendCmd("dev:all");
	}
	/**/
    //add dealing results and baseinfo
	netbase_msgs::netbase_msgs packet;
    double time2 = ros::Time::now().toSec();
    packet.stamp = ros::Time((time2 + time1) / 2.0);
    packet.data = std::string(ext);
    packet.ip = pack->ip;
    packet.port = pack->port;
	packet_pub.publish(packet);    //netbase_commubnicate_node订阅发布给上位机软件
}


void baseInfoCallback(const std_msgs::StringConstPtr &baseInfo)
{
	setBaseInfo(baseInfo->data);//define in zjudeal.cc
	//ROS_INFO("REC BASEINFO");
}

int main(int argc, char **argv)
{	
	geometry_msgs::Quaternion quaternions;
	geometry_msgs::Point point;

	ros::init(argc, argv, "set_nav_goals");
	ros::NodeHandle n;
    packet_pub = n.advertise<netbase_msgs::netbase_msgs>("netbase_server2user", 100);
    packet_sub = n.subscribe("netbase_user2server", 10, packetCallback);
	baseInfo_sub = n.subscribe("batteryAndRangeSensor", 2, baseInfoCallback);
	
	//declare objects
    SetNavGoal setNavGoal;
    ctrlLamp = new CtrlLamp;
	MoveCtrl moveCtrl;

	openCodeFile( (char *)"/home/robot/catkin_bjy/src/set_nav_goals/src/debug.txt" );
	ros::Rate loopRate(10);   //only 1???test
    
	initCmdDeal();
	
	while (ros::ok())
	{
		/**/
		if(setNavGoal.setTaskEnable == true && ( getStatus() == 1 ))
		{		
			if(ctrlLamp->setTaskEnable)
			{
  				char cmd_type ;
				unsigned int cmd_sub;
				unsigned int param[4];              
				int curPosetask = getActionTask(&cmd_type,&cmd_sub,param);  // nextActionTask
				if(curPosetask != 0 )//表示该点还有任务
				{				
					ROS_INFO("SET TASK!");
					ctrlLamp->sendCmd("dev:all");
					ctrlLamp->setTaskEnable = false;
					taskIsRunning = true;
					printf("%c%d\n",cmd_type,cmd_sub);
					printf("bb\n"); 
					int kkk = nextActionTask();	
				}
				else
				{
					taskIsRunning = false;   
					setNavGoal.setTaskEnable = false;
					printf("aa\n");
					if(nextActionPos()==2) {setStatus(-1);}  // lastpose
				}					
			}
			if(ctrlLamp->taskIsFininshed )
			{
				ROS_INFO("ONE TASK IS FINISHED!");
				ctrlLamp->setTaskEnable = true;
			}                         
		}
		if( ( deal.statusChange == 1 ) &&(deal.status == 2))
		{
			printf("bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb\n");
			deal.statusChange = 0;
			setNavGoal.cancelGoal();
			ctrlLamp->cancelGoal();
            ctrlLamp->sendCmd("dev:down");
		}/**/


		if( ( deal.statusChange == 1 ) &&(deal.status == 1))
		{
			printf("cccccccccccccccccccccccccc\n");
			deal.statusChange = 0;
		    setNavGoal.navResult = 1;
			taskIsRunning = false;
		}
		if((setNavGoal.navResult == 1) && ( getStatus() == 1 ) && ( taskIsRunning == false ) )
		{
			printf("ddddddddddddddddddddddddddddddddddddd\n");
			setNavGoal.navResult = 0;
			int x,y,z,r;
			double dx,dy,dz,dr;
			int ret = getActionPos( &x, &y, &z );
			
			dx = x/100.0; dy = y/100.0; dz = 0.0; dr = 0.0;
			printf("%f  %f  %f  \n",dx,dy,dz);
			setNavGoal.sendNavGoal(dx,dy,dz,dr);
			taskIsRunning = true;                     //////////////////////////add	
			ROS_INFO("SET GOAL");
		}
		//return current nav pose to PC
		setPosition((int)(setNavGoal.curx*100),(int)(setNavGoal.cury*100),(int)(setNavGoal.curz*100),(int)(setNavGoal.curr*100));
		ros::spinOnce();
		loopRate.sleep();
	}
	return 0;
}

