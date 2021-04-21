#include "base_node/base.h"

Base::Base(serial::Serial* serialPort):m_serialPort(serialPort)
{
    //load cfg param
    nh.param("/speed_limit/x",x_lim,1.0);
    nh.param("/speed_limit/y",y_lim,0.0);
    nh.param("/speed_limit/r",r_lim,0.8);
    nh.param("/serial1_cfg/RATE",rate,10);
    //init param about odom and tf
    initOdomParam();

    //pub and sub topic 
    odom_pub= nh.advertise<nav_msgs::Odometry>("odom", 2);   //发布里程计信息
    // set up a callbackQueue for velocityCallback
    // boost::function<void (const geometry_msgs::Twist::ConstPtr &)> velocityCallbackFun = boost::bind(&Base::velocityCallback, this,_1);
    velocitySubOptions = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        "cmd_vel",2,boost::bind(&Base::velocityCallback, this,_1),ros::VoidPtr(),&velocityCallbackQueue);
    velocity_sub = nh.subscribe(velocitySubOptions);

    //start a thread for getting odom and sending cmd_vel
    baseInfoThread = new boost::thread(boost::bind(&Base::baseInfoLoop, this));   //为里程计等数据的获取单独开一个线程
}

Base::~Base()
{
    if(baseInfoThread)
    {
        baseInfoThread->join();
        delete baseInfoThread;
        baseInfoThread = NULL;
    }
}

//init param about odom and tf
void Base::initOdomParam()
{
    //位置和速度的测量不确定性协方差矩阵第一组，两个选择一样,小车运动时使用
    float local_covariance1[36] = {1e-3, 0,   0,   0,   0,   0,  
                        0,   1e-3,0,   0,   0,   0,
                        0,   0,   1e6, 0,   0,   0,
                        0,   0,   0,   1e6, 0,   0,
                        0,   0,   0,   0,   1e6, 0,
                        0,   0,   0,   0,   0,   1e3};
    //位置和速度的测量不确定性协方差矩阵2,小车静止时使用
    float local_covariance2[36] = {1e-9, 0,   0,   0,   0,   0,  
                        0,   1e-3,1e-9,0,   0,   0,
                        0,   0,   1e6, 0,   0,   0,
                        0,   0,   0,   1e6, 0,   0,
                        0,   0,   0,   0,   1e6, 0,
                        0,   0,   0,   0,   0,   1e-9}; //静止时里程计的协方差比imu的小，数据融合更相信里程计的数据         
    for(int i = 0; i < 36; i++)
    {
        covariance1[i] = local_covariance1[i];
        covariance2[i] = local_covariance2[i];
    }
    pos_temp[0]=pos_temp[1]=pos_temp[2]=0;
    odom_point.x=odom_point.y=odom_point.z=0;
    odom_point_tf.x=odom_point_tf.y=odom_point_tf.z=0;
    orie=0;
    odom_quat = tf::createQuaternionMsgFromYaw(orie);   ////偏航角转换成四元数*/
    vel_linear.x=vel_linear.y=vel_linear.z=0;
    vel_angular.x=vel_angular.y=vel_angular.z=0;
    last_stamp=now_stamp=ros::Time::now();    
}


//实现char数组和short之间的转换
short Base::uint8_to_short( uint8_t *b )      //调用的是这个函数
{
    unsigned short ret;
    ret  = (unsigned short)(b[1]);
    ret |= (unsigned short)(b[0]) << 8;
    return (short)ret;
}


void Base::velocityCallback(const geometry_msgs::Twist::ConstPtr &cmd_input){
    double x,y,r;
    x = cmd_input->linear.x;
    y = cmd_input->linear.y;
    r = cmd_input->angular.z;
    if(fabs(x) > x_lim)
        x = x > 0 ? x_lim : -x_lim;
    if(fabs(y) > y_lim)
        y = y > 0 ? y_lim : -y_lim;
    if(fabs(r) > r_lim)
        r = r > 0 ? r_lim : -r_lim;

    unsigned char velx, vely,velr;
    velx = (signed char)(x * 100);         //m转化为cm
    vely = (signed char)(y * 100);
    velr = (signed char)(r * 180 / PI);  //rad转化为°
    ROS_INFO("%d",velx);
    send_buffer[0] = 0x52;
    send_buffer[1] = 0x54;
    send_buffer[2] = CMD_SPEEDSET;
    send_buffer[3] = 0x04;
    send_buffer[4] = 0x01;      //shenmeshenmemoshi
    send_buffer[5] = velx;
    send_buffer[6] = vely;      //y方向线速度
    send_buffer[7] = velr;      //角速度
    send_buffer[8] = send_buffer[4]+send_buffer[5]+send_buffer[6]+send_buffer[7];
    send_buffer[9] = 0x0A;
    send_buffer[10] = 0x0D;
    if(m_serialPort->write(send_buffer,11) != 11)
        ROS_ERROR("velcmd sent failed");
}



//向下位机请求里程计数据
void Base::odomRequest()
{
    send_buffer[0] = 0x52;
    send_buffer[1] = 0x54;
    send_buffer[2] = CMD_ODOMREQUEST;
    send_buffer[3] = 0x01;
    send_buffer[4] = 0x01;
    send_buffer[5] = 0x01;//clc
    send_buffer[6] = 0x0A;
    send_buffer[7] = 0x0D;
    if(m_serialPort->write(send_buffer,8) != 8)
        ROS_ERROR("Request sent failed");
}

//publish odom and tf
void Base::pubOdomAndTF()
{
    double dt=dt_Duration.toSec();  //.toSec() 将time类型时间戳转换为double类型数据
    dx = 1.0*uint8_to_short(receive_data+4)/100000.0;    //距离上一次申请里程计时间，机器人坐标系下x方向的位移，放大了100倍,mm;所以这里处理之后转化为m
    dy = -1.0*uint8_to_short(receive_data+6)/100000.0;
    dz = 1.0*uint8_to_short(receive_data+8)*PI/180000.0;    //角位移，放大了1000倍，°，所以这里处理之后转化为弧度,receive_data数组名当做指针再用

    //计算机器人坐标系下的速度
    vel_linear.x =( dx / dt + vel_linear.x) / 2.0;    //求解上一时刻和这一时刻的平均值
    vel_linear.y =( dy / dt + vel_linear.y) / 2.0;
    vel_angular.z =( dz / dt + vel_angular.z) / 2.0;


    /*将机器人坐标系下的位姿变化量转换到世界坐标系下*/
    pos_temp[0] += cos(orie) * dx - sin(orie) * dy;
    pos_temp[1] += sin(orie) * dx + cos(orie) * dy;

    //更新全局位姿态（注意是先更新XY位移，还是先更新方位角）
    orie += dz;
    orie = (orie > PI) ? (orie - 2*PI) : ((orie < -PI) ? (orie + 2*PI) : orie);


    //ROS_INFO("%f %f %f",pos_temp[0],pos_temp[1],orie);
    odom_point_tf.x=odom_point.x=pos_temp[0];
    odom_point_tf.y=odom_point.y=pos_temp[1];
    odom_quat = tf::createQuaternionMsgFromYaw(orie);   //偏航角转换成四元数
    //现在要用数据融合程序来发布这个tf变换，将发布odom_tf坐标变换这一行注释即可 
    //发布tf坐标变化
    odom_tf.header.stamp = ros::Time::now(); 
    odom_tf.header.frame_id = "odom";
    odom_tf.child_frame_id = "base_footprint";
    odom_tf.transform.translation = odom_point_tf;
    odom_tf.transform.rotation = odom_quat;        
    odom_bc.sendTransform(odom_tf);  //现在要用数据融合程序来发布这个tf变换，将发布odom_tf坐标变换这一行注释即可 
    
    //确定里程计的协方差矩阵，判断当前速度是否为0,若为0使用covariance2[36]，数据融合更相信里程计，否则使用covariance2[36]
    if((fabs(vel_linear.x) < 1e-6) && (fabs(vel_linear.y)<1e-6) && (fabs(vel_angular.z)<1e-6))
    {
        for(int i = 0; i < 36; i++){
            odom_inf.pose.covariance[i] = covariance2[i];
            odom_inf.twist.covariance[i] = covariance2[i];
        }
    }
    else
    {
        for(int i = 0; i < 36; i++){
            odom_inf.pose.covariance[i] = covariance1[i];
            odom_inf.twist.covariance[i] = covariance1[i];
        }
    }
    //发布里程计信息
    odom_inf.header.stamp = odom_tf.header.stamp; 
    odom_inf.header.frame_id = "odom";  //位置是在odom坐标系下的
    odom_inf.child_frame_id = "base_footprint"; //速度是在odom坐标系下的
    odom_inf.pose.pose.position= odom_point;
    odom_inf.pose.pose.orientation = odom_quat;       
    odom_inf.twist.twist.linear = vel_linear;
    odom_inf.twist.twist.angular = vel_angular;
    odom_pub.publish(odom_inf);
}

//publish 8 ultrasound sensor range info
void Base::pubOneSensor(ros::Publisher &ultrasound_pub,  const char *topic, const char *tfName, uint8_t rangeData)
{
    sensor_msgs::Range msg;
    std_msgs::Header header;
    header.stamp = now_stamp;
    header.frame_id = tfName;
    msg.header = header;
    msg.field_of_view = 25*PI/180;
    msg.min_range = 0.1;
    msg.max_range = 1.0;
    msg.range = rangeData/100.0;  //下位机传上来的的单位是cm
    ultrasound_pub = nh.advertise<sensor_msgs::Range>(topic, 2);
    ultrasound_pub.publish(msg);
    //ROS_INFO("sonar %d",rangeData);
}

void Base::pubRangeSensor()
{
    pubOneSensor(ultrasound_pub1, "sonar1", "ultrasound1", receive_data[10]);
    pubOneSensor(ultrasound_pub2, "sonar2", "ultrasound2", receive_data[11]);
    pubOneSensor(ultrasound_pub3, "sonar3", "ultrasound3", receive_data[12]);
    pubOneSensor(ultrasound_pub4, "sonar4", "ultrasound4", receive_data[13]);
    pubOneSensor(ultrasound_pub5, "sonar5", "ultrasound5", receive_data[14]);
    pubOneSensor(ultrasound_pub6, "sonar6", "ultrasound6", receive_data[15]);
    pubOneSensor(ultrasound_pub7, "sonar7", "ultrasound7", receive_data[16]);
    pubOneSensor(ultrasound_pub8, "sonar8", "ultrasound8", receive_data[17]);
}

//pub battery and rangesensor
void Base::pubBatteryAndRangeSensor()
{
    std_msgs::String msg;
    char baseInfo[512];
    //R is Rangensensor, V and P is battery info
    sprintf(baseInfo,"R%d R%d R%d R%d R%d R%d R%d R%d V%d P%d",
                    receive_data[10],receive_data[11],receive_data[12],receive_data[13],
                    receive_data[14],receive_data[15],receive_data[16],receive_data[17],
                    receive_data[18],receive_data[19]);
    msg.data = baseInfo;
    baseInfo_pub = nh.advertise<std_msgs::String>("batteryAndRangeSensor", 2);
    baseInfo_pub.publish(msg);

}

//get and publish odom\tf\sonar\battery information...etc
void Base::baseInfoLoop()
{  
    ros::Rate loop_rate(rate);
    while(ros::ok())
    {           //处理串口信息，计算并发布tf转换、odom_inf
        //请求下位机数据，并读取串口数据 
        odomRequest();
		m_serialPort->waitByteTimes(1000);
        while(!m_serialPort->waitReadable());
        m_serialPort->read(receive_data,sizeof(receive_data));

        if(receive_data[0]==0x52 && receive_data[1]==0x54 && receive_data[24]==0x0A && (receive_data[25]==0x0D || receive_data[25]==0x0A))
        { //校验
            //计算时间间隔并转化为秒
            now_stamp=ros::Time::now();
            dt_Duration=now_stamp-last_stamp;
            last_stamp=now_stamp;
            pubOdomAndTF();
            pubRangeSensor();
            pubBatteryAndRangeSensor();
            //ROS_INFO("BATTERY v  %d",receive_data[18]);//电压
            //ROS_INFO("BATTERY /  %d",receive_data[19]);//电量
        }
        else
        {
            ROS_ERROR("Fail to get odom data from STM32.");
        }
        //check velocityCallbackQueue and run velocityCallbackQueue
        velocityCallbackQueue.callOne(ros::WallDuration(0));
        loop_rate.sleep(); 
    }
}
