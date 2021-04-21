#include <string>
#include <cmath>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/file.h>
#include <ros/ros.h>
#include <netbase_communicate/netbase_communicate.h>
#include <netbase_msgs/netbase_msgs.h>


int socket_id;
#define	PACKET_SIZE	1560
int UDP_PORT_NUMBER;
char buf[PACKET_SIZE];

std::string frame_id;
ros::Publisher packet_pub;
ros::Subscriber packet_sub;
ros::Publisher vel_pub;

double notReceiveTimeBegin;
double notReceiveTimeEnd;
double notReceiveDuration;

bool loadParameters() 
{
    UDP_PORT_NUMBER = 1234;
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    return true;
}

bool openUDPPort() 
{
    UDP_PORT_NUMBER = 1234;            //ADD
    socket_id = socket(PF_INET, SOCK_DGRAM, 0);
    if (socket_id == -1) 
    {
        perror("socket");
        return false;
    }

    sockaddr_in my_addr;
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(UDP_PORT_NUMBER);
    ROS_INFO_STREAM("Opening UDP socket: port " << UDP_PORT_NUMBER);
    my_addr.sin_addr.s_addr = inet_addr("192.168.0.126");//INADDR_ANY;

    if (bind(socket_id, (sockaddr *)&my_addr, sizeof(sockaddr)) == -1) 
    {
        perror("bind");
        return false;
    }

    if (fcntl(socket_id, F_SETFL, O_NONBLOCK|FASYNC) < 0) 
    {
        perror("non-block");
        return false;
    }

    return true;
}

void packetCallback(const netbase_msgs::netbase_msgs::ConstPtr &pack)
{
    printf("S:%s %d bytes!\n",pack->data.c_str(),(int)pack->data.length());

    sockaddr_in my_addr;
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(pack->port);
    my_addr.sin_addr.s_addr = pack->ip;

    void *msg = (void *)pack->data.c_str();
    unsigned int len = pack->data.length();
    ssize_t bytes_sent=sendto(socket_id, msg, len, 0,(struct sockaddr *)&my_addr,sizeof(struct sockaddr_in));
}

bool createRosIO() 
{
    // Output
	ros::NodeHandle nh;
	ros::NodeHandle pnh;
    packet_pub = nh.advertise<netbase_msgs::netbase_msgs>("netbase_user2server", 100);
    packet_sub = nh.subscribe("netbase_server2user", 100,packetCallback);
    vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    return true;
}

bool zjuagv_initialize() 
{
    if (!loadParameters()) 
    {
        ROS_ERROR("ROS parameters error");
        return false;
    }

    if (!createRosIO()) 
    {
        ROS_ERROR("ROS createRosIO error");
        return false;
    }

    if (!openUDPPort()) 
    {
        ROS_ERROR("Cannot open UDP port");
        return false;
    }
    ROS_INFO("Initialised zju agv success");
    return true;
}

int getPacket(/*netbase_msgs *packet*/) 
{
    double time1 = ros::Time::now().toSec();

    struct pollfd fds[1];
    fds[0].fd = socket_id;
    fds[0].events = POLLIN;
    static const int POLL_TIMEOUT = 10;

    sockaddr_in sender_address;
    socklen_t sender_address_len = sizeof(sender_address);
    netbase_msgs::netbase_msgs packet;


    do {
        int retval = poll(fds, 1, POLL_TIMEOUT);

        if (retval < 0)
        {
            if (errno != EINTR)
            {
                ROS_ERROR("poll() error: %s", strerror(errno));
            }
            return 1;
        }

        if (retval == 0)
        {
            /*计算当前未接收到上位机信息的时间，若超过1.5s,则作出异常处理，robot首次连接时不会触发(因为有receiveErrorDone标志来限制，初始值为true)*/
            notReceiveTimeEnd = ros::Time::now().toSec();
            notReceiveDuration = notReceiveTimeEnd - notReceiveTimeBegin;
            if(notReceiveDuration>2 && receiveErrorDone==false) //正常工作中，若出现超过2s网络连接断开，则处理异常
            {       
                dealReceiveError();
                receiveErrorDone = true;
                ROS_ERROR("Don't Receive Anything From UpPC!"); //正常情况下，上位机软件通过定时器每隔500ms向该节点发送数据(UDP通信)
            }
            return 1;
        }

        if (retval == 1)
        {
            //ROS_INFO("Receive Data From UpPC!");
            //若接收到上位机的消息，则重新设定notReceiveDuration的开始时刻; 初始值设定是在 zjuagv_polling()函数中进行的
            notReceiveTimeBegin = ros::Time::now().toSec();
            receiveErrorDone = false;
            
        }

        if ((fds[0].revents & POLLERR) || (fds[0].revents & POLLHUP) || (fds[0].revents & POLLNVAL))
        {
            ROS_ERROR("poll() reports msg error");
            return 1;
        }
    } while ((fds[0].revents & POLLIN) == 0);

    ssize_t nbytes = 0;
    nbytes = recvfrom(socket_id, buf, PACKET_SIZE,  0, (sockaddr*) &sender_address, &sender_address_len);
    buf[nbytes] = 0;

    if (nbytes < 0)
    {
        if (errno != EWOULDBLOCK)
        {
            perror("recvfail");
            ROS_INFO("recvfail");
            return 1;
        }
    }

    sockaddr_in *pSendAddr = (sockaddr_in *)&sender_address;
    double time2 = ros::Time::now().toSec();
    packet.stamp = ros::Time((time2 + time1) / 2.0);
    packet.data = std::string(buf);
    packet.ip = pSendAddr->sin_addr.s_addr;
    packet.port = htons(pSendAddr->sin_port);
    printf("R:%s\n",buf);
    //printf("port:%d ip:%s\n",htons(pSendAddr->sin_port),(char*)inet_ntoa(pSendAddr->sin_addr));

    packet_pub.publish(packet);
    return 0;
}



bool zjuagv_polling()
{
    notReceiveTimeBegin = ros::Time::now().toSec();
    while (ros::ok())
    {
        int rc = getPacket(/*&packet*/);
        if (rc == 0)    break;
        if (rc < 0)     return false;
    }

    ROS_DEBUG("Publishing a full msgs.");

    return true;
}

void dealReceiveError()
{
    cmd_vel.linear.x = 0;
    cmd_vel.angular.z = 0;
    vel_pub.publish(cmd_vel); 
}

