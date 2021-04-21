#include <iostream>
#include <base_controller/base_controller.h>
#include <fstream>
#include <std_msgs/String.h>
#include <std_msgs/UInt32.h>
#include <string>
#include "serial/serial.h"
#include <ros/ros.h>
#include <string>        
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include <math.h>
#include "serial/serial.h"
using namespace ros;
using namespace std;

//#define DEBUG_RS485_BATTERY_VOLTAGE_ACQUIRE 

//#define BATTERY_ACCESS_NEW_METHOD

#define PI 3.14159
unsigned short byte_to_uint16( byte *b )
{
    unsigned short ret;
    ret  = (unsigned short)(ByteCast(b[1]));
    ret |= (unsigned short)(ByteCast(b[0])) << 8;
    return ret;
}
short byte_to_int16( byte *b )
{
    return (short)byte_to_uint16(b);
}

base::base()//:
//DGConsole("base",boost::bind(&base::processCmd, this, _1)),
//    private_nh("~")
{
    baseThread = NULL;
    robot_poseR_=0.0;
    b_imu_first_data_=true;

    poseX = poseY = poseR = 0.0;
    velX=velY=velR= 0.0;
    detect_count=0;

#ifdef TEST_ODOM_CHECK  
    private_nh.param("printOdomFromCtrlBrd", printOdomFromCtrlBrd, false);
    testOdomAxisX = 0.0;    
    testOdomAxisY = 0.0;    
    testOdomDegreeR = 0.0;
#endif

	//rostopic  pub -1 /userSub std_msgs/String -- "dev:check"
    sleepState = false;
    imu_w=0.0;
    //imuSub = nh.subscribe<sensor_msgs::Imu>("/imu/data", 10, &base::handleImuData, this);
    //////////cmdSub = nh.subscribe<std_msgs::String>("base_cmd", 10, &base::onCmd, this);
    userSub = nh.subscribe<std_msgs::String>("userSub", 10, &base::onCallBackUser, this);
    //////////TwistSub = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1,  &base::callBackCmdVel, this);
    //////////robotPoseSub = nh.subscribe<geometry_msgs::PointStamped> ("robot_pose", 1, &base::onUpdateRobotPose,this);

    //baseInfoPub = nh.advertise<dg_msgs::BaseInfo>("base_info", 10);
    //batteryInfoPub = nh.advertise<dg_msgs::BaseInfo>("battery_info", 10);
    //////////audioPub = nh.advertise<std_msgs::String>("audio_string",1);
    //////////odomPub = nh.advertise<nav_msgs::Odometry>("odom", 10);
    //statusPub=nh.advertise<dg_msgs::NodeStatus>("nodes_status",10);
    //////////naviCmdPub = nh.advertise<std_msgs::String>("navigation_cmd", 1);
    //////////socketCmdPub = nh.advertise<std_msgs::String>("socket_cmd", 10);
    //sleepstateCall= nh.serviceClient<dg_msgs::naviState>("laser_msg_srv");
    baseUserPub = nh.advertise<std_msgs::UInt32>("baseUserPub", 10);

    private_nh.param("baseComName", baseComName, string("/dev/ttyS0"));
    //private_nh.param("sonarComName", sonarComName, string("/dev/ttyS3"));
    private_nh.param("baseUserName", baseUserName, string("/dev/ttyUSB1"));
    private_nh.param("LoopRate", loopRate, 25);
    private_nh.param("initR", initR, 0.0);
    
    private_nh.param("use_auto_sleep",  b_auto_sleep, false);
    private_nh.param("modify_odom", b_modify_odom_, false);
    private_nh.param("maxLineSpeed", maxLineSpeed, 1.2);
    private_nh.param("maxTurnSpeed", maxTurnSpeed, 2.2);
    private_nh.param("use_imu", b_use_imu_, false);
    private_nh.param("imu_offset", b_imu_offset, true);
    //LOGS_INFO("base_node")<<"use_imu:"<<b_use_imu_;
    //LOGS_INFO("base_node")<<"imu_offset:"<<b_imu_offset;
    private_nh.param("useSpeedSmooth", useSpeedSmooth, true);
    private_nh.param("useJoyStickSpdTest", useJoyStickSpdTest, false);
    private_nh.param("speedStepBig", speedStepBig, 0.2);
    private_nh.param("speedStepSmall", speedStepSmall, 0.02);
    private_nh.param("spdKeepTimeJoystick", spdKeepTimeJoystick, 10);
        
    private_nh.param("comFile", comFile, string(""));
    private_nh.param("use_sonar",use_sonar, false);
    private_nh.param("new_battery",new_battery, false);


    private_nh.param("zeroSpdRpeatTimes", zeroSpdRpeatTimes, 3);
    bBrakeon=false;
    //baseCom = new serial::Serial(baseComName.c_str(), 9600, serial::Timeout::simpleTimeout(1000)); //配置串口
    //if( baseCom->isOpen() == true )
    {
        //LOGS_INFO("base_node")<<"baseCom open succ!";
        ////LOGS_INFO("baseCom open succ!");
        //baseThread = new boost::thread(boost::bind(&base::baseLoop, this));
    }
    //else
    {
        //LOGS_ERROR("base_info")<<"baseCom open fail!";
        ////LOGS_ERROR("baseCom open fail!");
    }

    baseUser = new serial::Serial(baseUserName.c_str(), 9600, serial::Timeout::simpleTimeout(1000)); //配置串口
    //if( baseUser->isOpen() == true )
	{
    //if(batteryCom.OpenAs485(batteryComName.c_str(), O_RDWR))//  �Լ�  O_NOCTTY �� O_NDELAY
    //{
    //    batteryCom.SetAs485(9600, 0, 8, 1, 'n', 1);
    //     LOGS_INFO("base_node")<<"battery thread enabled, and batteryCom open succ!";

        //LOGS_INFO("battery thread enabled, and batteryCom open succ!");
        batteryThread = new boost::thread(boost::bind(&base::batteryLoop, this));
    //}
    //else
    //{
    //    LOGS_ERROR("base_node")<<"battery thread enabled, but batteryCom open fail!";
        //LOGS_ERROR("battery thread enabled, but batteryCom open fail!");
    //}
	}
}

base::~base()
{
    if(baseThread)
    {
        baseThread->join();
        delete baseThread;
        baseThread = NULL;
        //LOGS_INFO("base_node")<<"baseThread stop!";
    }
	delete baseCom;
}

void base::onUpdateRobotPose(const geometry_msgs::PointStampedConstPtr &data)
{
    boost::lock_guard<boost::shared_mutex> lck(robot_pos_mx_);
    robot_poseR_ = data->point.z;
	if(abs(initR)<0.001)initR=robot_poseR_;
}

void base::onCmd(const std_msgs::StringConstPtr &cmd)
{
    processCmd(cmd->data.c_str());
}

void base::callBackCmdVel(const geometry_msgs::TwistConstPtr &vel)
{
    if(vel->linear.z > 0.000001)
    {
        Speed(0,0,0);
    }
    else
    {
        Move( vel->linear.x, vel->linear.y, vel->angular.z );
    }
}

void base::Move(float x, float y, float r, bool delay /*= false*/)
{
    if(abs(x) > maxLineSpeed)
	{
        x = x > 0 ? maxLineSpeed : -maxLineSpeed;
	}
    if(abs(y) > maxLineSpeed)
	{
        y = y > 0 ? maxLineSpeed : -maxLineSpeed;
	}
    if(abs(r) > maxTurnSpeed)
	{
        r = r > 0 ? maxTurnSpeed : -maxTurnSpeed;
	}

    if(useSpeedSmooth)
    {
        if(abs(x-velX) < 0.05 && abs(y - velY) < 0.05)
        {
            Speed(x, y, r);
        }
        else
        {
            double speedStep = speedStepSmall;
            int    spdKeepTime = 32 * 1000;
            if(useJoyStickSpdTest)
            {
                speedStep = speedStepBig;
                spdKeepTime = spdKeepTimeJoystick*1000;
            }

            float temp_x = velX;
            float temp_y = velY;
            int speedcountx = abs(x - velX) / speedStep;
            int speedcounty = abs(y - velY) / speedStep;
            int speedcount = speedcountx > speedcounty ? speedcountx : speedcounty;
            for(int i = 0; i < speedcount; i++)
            {
                temp_x = x > temp_x ? temp_x + speedStep : (x < temp_x ? temp_x - speedStep : x);
                temp_y = y > temp_y ? temp_y + speedStep : (y < temp_y ? temp_y - speedStep : y);
                Speed(temp_x, temp_y, r);
                usleep(spdKeepTime);
            }
            Speed(x, y, r);
        }
    }
    else
    {
        Speed(x, y, r);
    }
}

void base::Speed(float x, float y, float r, bool delay /*= false*/)
{
    int sendtimes;

    if(bBrakeon)
	{
     	x=0;y=0;r=0;
    }
    if(abs(x) > maxLineSpeed)
	{
        x = x > 0 ? maxLineSpeed : -maxLineSpeed;
	}
    if(abs(y) > maxLineSpeed)
	{
        y = y > 0 ? maxLineSpeed : -maxLineSpeed;
	}
    if(abs(r) > maxTurnSpeed)
	{
        r = r > 0 ? maxTurnSpeed : -maxTurnSpeed;
	}

    int8 velx, vely,velr;
    velx = (int8)(x * 100);
    vely = (int8)(y * 100);
    velr = (int8)(r * 180 / PI) / 2.0;

    velX = x;
    velY = y;
    velR = r;
    byte cmd[CMD_LEN];
    size_t len = GetControlCmd((delay ? 0x02 : 0x01), velx, vely, velr, cmd);

    sendtimes = 1;
    if ((velx == 0) &&(vely == 0) &&(velr == 0))
    {
        sendtimes = zeroSpdRpeatTimes;
    }
    for( ; sendtimes > 0; sendtimes--)
    {
        //LOGS_DEBUG("base_node")<<"Move:"<< velx<<" "<< vely<<" "<<velr;
		if(sendtimes>1)usleep(100000);
        size_t writeLen = baseCom->write(cmd, len);
        if(len != writeLen)
        {
            char str[1024];
            int cur = 0;
            for(int i = 0; i < len; i++)
            {
                cur += sprintf(str + cur, "%02X ", cmd[i]);
            }
            str[len] = '\0';
            //LOGS_ERROR("base_node")<<"(to baseCom)Send Error! Cmd type [MOVE-SPEED]";
            //LOGS_ERROR("base_node")<<" Data len , send len:"<< len<<" "<<writeLen;
            //LOGS_ERROR("base_node")<<" Raw data:"<< str;
        }
    }
}

void base::processCmd(const char *cmd)
{
#if	0
    int d1,d2;
    float f1, f2, f3;
    if(PEEK_CMD(cmd, "exit"))
        ros::requestShutdown();
    else if(PEEK_CMD(cmd, "chargeon"))
    {
        //LOGS_INFO("base_node")<<"recv command chargeon , send IO(0x66) to baseboard";
         if(0==m_boardAckBits[4])
		 {
			 IO(0x66);
			 sleep(1);
			 //IO(0x69);
		 }
         else
         {
             //LOGS_WARN("base_node")<<"charging begin";
         }
  
    }
    else if(PEEK_CMD(cmd, "chargeoff"))

#endif
}

byte base::CRC8( const byte *buf, byte len)
{
  byte crc = 0;
  byte i;
  
  for(i=0; i<len; i++)
  {
    crc += buf[i];
  }
  crc^=0xff;
  return crc;
}

byte  base::CheckCRC(byte *crctmp, byte len)
{
    byte crc_result;
    byte crc_resultL,crc_resultH;
        
    crc_result = CRC8(crctmp, len-3);
    crc_resultL = crc_result & 0X0F;
    crc_resultH = (crc_result & 0XF0) >> 4;
    //crc_resultH = hexNumToAsciiNum(crc_resultH);
    //crc_resultL = hexNumToAsciiNum(crc_resultL);              
    return 0;
}

void base::baseLoop()
{
    //LOGS_INFO("base_node")<<"Base Thread Start... ";
    ros::Rate baseLoopRate(loopRate);
    
    byte cmd[CMD_LEN];
    memset((void*)cmd, 0, CMD_LEN);
    size_t cmdLen = GetQueryCmd(cmd);
    byte recvBuffer[100];
    byte dataBuffer[100];
    size_t size = 0;
    size_t len = 0;
    int re;
    fd_set  fdSlave;
    int m_port_file ;//= baseCom.getComHandle();
	int maxFd = m_port_file + 1;
    struct timeval tv;
    int dataLen = 16;

    batt_thread_flag_=false;
    baseLoop_thread_flag_=false;


   /* sleepNeedWakeup = IsNeedWakeUp();
    if(sleepNeedWakeup)
    {
        SendWakeupCmd();
    }*/
    
    ros::Time lastTime = ros::Time::now();
    while(ros::ok() && loopRate)
    {
        ////LOGS_INFO("base_node")<<"send (odom,status)feedback request to base board";
        baseLoop_thread_flag_=true;
        //if(baseCom.write(cmd, cmdLen) == cmdLen)
        {   
            FD_ZERO(&fdSlave);
            FD_SET(m_port_file, &fdSlave);
            while(1)
            {
                FD_ZERO(&fdSlave);
                FD_SET(m_port_file, &fdSlave);
                tv.tv_sec = 0;
                tv.tv_usec = 600 * 1000;
                re = select(maxFd, &fdSlave, NULL, NULL, &tv);
                // //LOGS_INFO("base_node")<<"re "<<re;
                if(re > 0)
                {
                    if(FD_ISSET(m_port_file,&fdSlave) )
                    { 
                        size ;//= baseCom.read(recvBuffer, 100);
                        memcpy(dataBuffer+len, recvBuffer, size);
                        len += size;
                        ////LOGS_INFO("size %d,len %d",size,len);
                        if(len >= dataLen && dataBuffer[len - 2] == 0x0A && (dataBuffer[len - 1] == 0x0D || dataBuffer[len - 1] == 0x0A))
                        {
                            int base = 0;
                            if(len > dataLen) 
                            {
                                base = len - dataLen;
                            }
                            byte* cmd = dataBuffer + base;

                            if(cmd[0] == 0x52 && cmd[1] == 0x54 && cmd[14] == 0x0A &&(cmd[15] == 0x0D || cmd[15] == 0x0A))
                            {
								#ifdef __DEBUG__H
                                char ss[128];
                                int offset = 0;
                                for(int i = 0; i < 16; i++) 
								{
                                    offset += sprintf(ss + offset, "%02X ", cmd[i]);
                                }
                                //LOGS_WARN("base_node")<<" recv from baseCom, data(Raw):"<< ss;
								#endif
                                      

                                int  userDatBytes = cmd[3];
                                byte crc = 0;
                                for(int i = 4; i < 4 + userDatBytes; i++) 
                                {
                                    crc += cmd[i];
                                }
                                
                                if(crc != cmd[13])
                                {
                                    //LOGS_ERROR("base_node")<<"CRC Error!!(data from baseCom crc error)";
                                }
                                else
                                {
                                    int val;
                                    double x, y, z;
                                    byte statusHighByte;//cmd[11];
                                    byte statusLowByte;//cmd[12];
                                    
                                    statusHighByte = cmd[11];
                                    statusLowByte = cmd[12];
                                    processStatusInfo(statusHighByte, statusLowByte);
                                    
                                    ProcessACK(cmd[10]);
                                    
                                    val = byte_to_int16(cmd + 4);
                                    x = val / 100000.0f;
                                    val = byte_to_int16(cmd + 6);
                                    y = val / 100000.0;
                                    
                                    if(b_use_imu_)
                                    {
                                        z=imu_w;
                                    }
                                    else
                                    {
										val = byte_to_int16(cmd + 8);
                                        z = val / 1000.0f;
                                    }

                                    //dg_msgs::NodeStatus status_;
                                    //status_.comment.data="";
                                    //if(baseLoop_thread_flag_)
                                    //{
                                    //      status_.statusCode=0;
                                    //      status_.nodeName.data="base";
                                    //      statusPub.publish(status_);
                                    //}
                                    batt_thread_flag_=false;
                                    baseLoop_thread_flag_=false;

            						#ifdef TEST_ODOM_CHECK    
                                    if(printOdomFromCtrlBrd)
                                    {
                                        testOdomAxisX += x;    
                                        testOdomAxisY += y;  
  										if(b_use_imu_)
										{
					 					 	testOdomDegreeR = z;
										}  
                                        else testOdomDegreeR += z;
                                        
                                        //Seven//LOGS_WARN("test Odom Axis X = %f meters", this->testOdomAxisX);
                                        //Seven//LOGS_WARN("test Odom Axis Y = %f meters", this->testOdomAxisY);
                                       //LOGS_WARN("base_node")<<"Axis X ="<< this->testOdomAxisX;
                                       //LOGS_WARN("base_node")<<"Axis Y ="<< this->testOdomAxisY;
                                       //LOGS_WARN("base_node")<<"Odom  R:"<<this->testOdomDegreeR<<" degree, mod by 360: "<<  ((int)(this->testOdomDegreeR)) % 360<<" degree"<<",offset:"<<imu_offset;
                                        //Seven//LOGS_WARN("test Odom Axis X = %f meters", this->testOdomAxisX);
                                        //Seven//LOGS_WARN("test Odom Axis Y = %f meters", this->testOdomAxisY);

                                    }
            						#endif
                                    z = z * 3.1415926 / 180.0;
                                    ros::Time currentTime = ros::Time::now();
                                    float interval = currentTime.toSec() - lastTime.toSec();
                                    lastTime = currentTime;
                                    velocityX =( x / interval + velocityX) / 2.0;
                                    velocityY =( y / interval + velocityY) / 2.0;
                                    if(b_use_imu_)
                                    {
                                         velocityR =imu_Velz* 3.1415926 / 180.0 ;
                                    }
                                    else
                                    {
                                         velocityR =( z / interval + velocityR) / 2.0;
                                    }

									if(b_use_imu_)
									{
									 	this->poseR=z;
									}  
									else this->poseR += z;;
                                    //
                                    
                                    this->poseR = (this->poseR > PI) ? (this->poseR - 2*PI) : ((this->poseR < -PI) ? (this->poseR + 2*PI) : this->poseR);
                                    ////LOGS_INFO("base_node")<<"robot_poseR_:"<<(float)robot_poseR_-initR<<",poseR:"\
                                    //                     <<(float)poseR;
                                    static bool b_update_R=true;
                                    static float acc_odom_x=0.0;
                                    static float acc_odom_y=0.0;
                                    acc_odom_x+=x;
                                    acc_odom_y+=y;
                                     if(b_modify_odom_&&b_update_R&&abs(velocityX)<0.00001&&abs(velocityY)<0.00001&&abs(velocityR)<0.0001&&\
                                             abs(robot_poseR_-initR-poseR)>0.01*PI&&(robot_poseR_*this->poseR)!=0\
											 &&abs(robot_poseR_-initR-poseR)<0.2*PI
                                        )
                                     {
                                        // this->poseX += cos(robot_poseR_-initR) * x - sin(robot_poseR_-initR) * y;
                                        // this->poseY += sin(robot_poseR_-initR) * x + cos(robot_poseR_-initR) * y;
                                         //LOGS_WARN("base_node")<<"robot_poseR_:"<<(float)robot_poseR_-initR<<",poseR:"\
                                                              <<(float)poseR;
                                         poseR=robot_poseR_-initR;
                                         b_update_R=false;
                                         acc_odom_x=0.0;
                                         acc_odom_y=0.0;
										 this->poseX += cos(this->poseR) * x - sin(this->poseR) * y;
                                         this->poseY += sin(this->poseR) * x + cos(this->poseR) * y;
                                     }
                                     else if(b_modify_odom_&&b_update_R&&abs(velocityR)<0.001&&(abs(acc_odom_x)>2.0||abs(acc_odom_y)>2.0)\
										&&(robot_poseR_*this->poseR)!=0\
										&&abs(robot_poseR_-initR-poseR)<0.2*PI
										)
                                     {
                                        // this->poseX += cos(robot_poseR_-initR) * x - sin(robot_poseR_-initR) * y;
                                        // this->poseY += sin(robot_poseR_-initR) * x + cos(robot_poseR_-initR) * y;
                                         //LOGS_WARN("base_node")<<"acc_odom_x--->robot_poseR_:"<<(float)robot_poseR_-initR<<",poseR:"\
                                                              <<(float)poseR;
                                         poseR=robot_poseR_-initR;
                                         b_update_R=false;
                                         acc_odom_x=0.0;
                                         acc_odom_y=0.0;
										 this->poseX += cos(this->poseR) * x - sin(this->poseR) * y;
                                         this->poseY += sin(this->poseR) * x + cos(this->poseR) * y;
                                     }
                                     else
                                     {
                                          this->poseX += cos(this->poseR) * x - sin(this->poseR) * y;
                                          this->poseY += sin(this->poseR) * x + cos(this->poseR) * y;

                                     }
                                     if(abs(velocityX)>0.1)
                                     {
                                      b_update_R=true;
                                     }

                                     /*
                                    //LOGS_WARN("base_node")<<"robot_poseR_:"<<(float)robot_poseR_-initR<<",poseR:"\
                                                         <<(float)poseR;
                                    this->poseX += cos(this->poseR) * x - sin(this->poseR) * y;
                                    this->poseY += sin(this->poseR) * x + cos(this->poseR) * y;
                                      */
                                    

                                /*     if(abs(velocityX)<0.001&&abs(velocityY)<0.001&&velocityR<0.001&&abs(robot_poseR_-this->poseR)>0.1*PI)
                                     {
                                             this->poseR=robot_poseR_-initR;
                                     }
                                */
                                    
                                    odom_pub();
                                }
                            }//end else if cmd=cmd
                            else
                            {
                                //LOGS_ERROR("base_node")<<"Receive wrong baseCom feedback data! Head - End (5)";
                                char str[100];
                                int t = 0;
                                for(int i = 0; i < len ; i ++)
								{
                                    t += sprintf(str + t,"0x%02X ", dataBuffer[i]);
								}
                                 //LOGS_ERROR("base_node")<< str;
                            }//end else if cmd=cmd

                            len = 0;
                            memset(dataBuffer, 0, 100);
                            break;  
                        }//end if len>datalen
                    }//if FD_ISSET
                }//re >0
                else if(re == 0 )
                {
                    //str.data="the baseCom is disconnected!";
                    //audioPub.publish(str); 
                    //LOGS_ERROR("base_node")<<"Receive baseCom feedback time out!";
                    break;
                }
                else
                {
                    //LOGS_ERROR("base_node")<<"poll read baseCom error!";
                    break;
                }// end re
            }//end while(1)
        }//end if cmdwrite
        //else 
        {
            //LOGS_ERROR("base_node")<<"baseCom send feedback cmd error!";
            //baseCom.close();
            usleep(500000);
            //if(baseCom.open())
                //LOGS_WARN("base_node")<<"Rest baseCom port!";
           // m_port_file = baseCom.getComHandle();
        }//end if-else cmdwrite
                                    
        baseLoopRate.sleep();
    }//end while loop
    
    //LOGS_ERROR("base_node")<<"Base Thread Stop.";
    //dg_msgs::NodeStatus status_;
    //status_.comment.data="Base Thread Stop";
    //status_.statusCode=1;
    //status_.nodeName.data="base";
    //statusPub.publish(status_);

}

void base::onCallBackUser(const std_msgs::StringConstPtr &cmd)
{
    ROS_INFO("onCallBackUser");
    processCmd(cmd->data.c_str());
	if( strcmp( cmd->data.c_str(), "dev:up") == 0 )
	{
    	ROS_INFO("onCallBackUser:1");
		userCmd = 1;
	}
	else if( strcmp( cmd->data.c_str(), "dev:down") == 0 )
	{
    	ROS_INFO("onCallBackUser:2");
		userCmd = 2;
	}
	else if( strcmp( cmd->data.c_str(), "dev:all") == 0 )
	{
    	ROS_INFO("onCallBackUser:3");
		userCmd = 3;
	}
	else if( strcmp( cmd->data.c_str(), "dev:check") == 0 )
	{
    	ROS_INFO("onCallBackUser:0");
		userCmd = 0;
	}
}

#define	BIT_SENSOR_S1		0X01000000
#define	BIT_SENSOR_S2		0X02000000

#define	BIT_ACK_CMD			0X80000000
#define	BIT_ACK_STATUS		0X40000000

#define	MASK_MOTO_DEAL		0X0000FF00
#define	MASK_WORK_DEAL		0X00FF0000
#define	MASK_ERROR			0X000000FF

#define	STR_CMD_DEV_UP		"dev:up"
#define	STR_CMD_DEV_DWON	"dev:down"
#define	STR_CMD_DEV_WORK	"dev:work"

void base::batteryLoop()
{ 
    ROS_INFO("base_node");
    ros::Rate LoopRate(10);
    while(ros::ok()&& loopRate)
	{
		if(userCmd != -1 )
		{
    		byte cmd[12]={0XFE,0XEE,0X05,0X01,0X00,0X03,0X00,0X00,0X55,0X55,0XFC,0XFF};
			cmd[3] = userCmd;
    		byte rcv[1024];
			userCmd = -1;
    		ROS_INFO("deal userCmd");

    		bool ret;
			int sizeRcv;
			baseUser->waitByteTimes(1000);
			//sizeRcv = baseUser->read(rcv,sizeof(rcv));
			if(baseUser->write(cmd, sizeof(cmd)) == sizeof(cmd))
			{
                ret = baseUser->waitReadable();
                if(ret == true)
                {
    				ROS_INFO("true");
					sizeRcv = baseUser->read(rcv,sizeof(rcv));
					if(sizeRcv < 12 )
					{
						ROS_INFO("error rcv");
					}
					else
					{
						if(( rcv[0] == 0xfe )&&( rcv[1] == 0xee )&&( rcv[10] == 0xfc )&&( rcv[11] == 0xff ))
						{
							std_msgs::UInt32 aaaaa;
							aaaaa.data = rcv[7];
							baseUserPub.publish(aaaaa);
    						ROS_INFO("ack success");
						}
					}
				}
				else
				{
    				ROS_INFO("false");
					std_msgs::UInt32 aaaaa;
					aaaaa.data = 0;
					baseUserPub.publish(aaaaa);
    				ROS_INFO("1");
				}
			}
		}
        LoopRate.sleep();
	}

}

void base::ProcessACK(byte ack)
{
    for(int i=0;i<8;i++)
    {
        m_boardAckBits[i]=ack&(1<<i);
    }
    static int cnt_printAck=0;
	if(cnt_printAck++>1000)
    {
      //LOGS_INFO("base_node")<<"ProcessACK ack bits:"<< m_boardAckBits[0]<<" "<<m_boardAckBits[1]<<" "<<m_boardAckBits[2]<<" "<<m_boardAckBits[3]\
                         <<" "<<m_boardAckBits[4]<< " "<<m_boardAckBits[5]<<" "<<m_boardAckBits[6]<<" "<<m_boardAckBits[7];
       cnt_printAck=0;
    }

    static int check_sleep_state_cnt_=0;
	if(m_boardAckBits[4]==0)

    {
        auto_awake_laser_=0;
    }
    if(auto_awake_laser_<100&&b_auto_sleep&&(m_boardAckBits[4]==1)&&(m_boardAckBits[0]==0)&&(check_sleep_state_cnt_++ >100))
    {
        check_sleep_state_cnt_=0;
		auto_awake_laser_=150;
    }
     else if(auto_awake_laser_++>400||\
            (b_auto_sleep&&(m_boardAckBits[4]==0)&&(m_boardAckBits[0]==1)\
             &&(check_sleep_state_cnt_++ >100)))
    {
        check_sleep_state_cnt_=0;
		auto_awake_laser_=150;
    }

}

void base::processStatusInfo(byte statusHighByte, byte statusLowByte)
{

}

size_t base::GetControlCmd(int8 moveType, int8 vel_1, int8 vel_2, int8 vel_3, byte *cmd)
{
    memset((void*)cmd, 0, CMD_LEN);
    cmd[0] = 0x52;
    cmd[1] = 0x54;

    cmd[2] = 0x01;
    cmd[3] = 0x04;

    cmd[4] = moveType;
    cmd[5] = vel_1;
    cmd[6] = vel_2;
    cmd[7] = vel_3;

    cmd[8] = cmd[4] + cmd[5] + cmd[6] + cmd[7];
    cmd[9] = 0x0A;
    cmd[10] = 0x0D;
    return 11;
}

size_t base::GetQueryCmd(byte *cmd)
{
    memset((void*)cmd, 0, CMD_LEN);
    cmd[0] = 0x52;
    cmd[1] = 0x54;

    cmd[2] = 0x02;
    cmd[3] = 0x01;

    cmd[4] = 0x01;

    cmd[5] = cmd[4];
    cmd[6] = 0x0A;
    cmd[7] = 0x0D;

    return 8;
}

void base::odom_pub()
{
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(poseR);
    geometry_msgs::TransformStamped odom_trans;
    
    odom_trans.header.stamp = ros::Time::now();
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    odom_trans.transform.translation.x = poseX;
    odom_trans.transform.translation.y = poseY;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
    tfBroadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time::now();
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";

    odom.pose.pose.position.x = poseX;
    odom.pose.pose.position.y = poseY;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    odom.twist.twist.linear.x = velocityX;
    odom.twist.twist.linear.y = velocityY;
    odom.twist.twist.angular.z = velocityR;
    odomPub.publish(odom);
}


