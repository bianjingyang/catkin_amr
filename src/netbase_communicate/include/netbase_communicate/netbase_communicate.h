#ifndef NETBASE_COMMUNICATE_H_
#define NETBASE_COMMUNICATE_H_

#include <unistd.h>
#include <stdio.h>
#include <netinet/in.h>
#include <string>
#include <ros/ros.h>
#include <netbase_msgs/netbase_msgs.h>
#include <geometry_msgs/Twist.h>


bool zjuagv_initialize();
bool zjuagv_polling();
static void packetCallback(const netbase_msgs::netbase_msgs::ConstPtr& pack);

bool loadParameters();
bool createRosIO();
bool openUDPPort();
int getPacket(/*netbase_msgs *msg*/);
void dealReceiveError();
static bool receiveErrorDone = false;

static geometry_msgs::Twist cmd_vel;




#endif // NETBASE_COMMUNICATE_H_
