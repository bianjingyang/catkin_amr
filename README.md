--------------------------------------------------
##独立启动各个功能的指令 amr_bringup
--------------------------------------------------
超声波右前方为1 左前方为2  右后方为3 左后方为4

*__最开始编译工作空间执行顺序__
   将range_sensor_layer中的RangeSensorLayer.cfg文件 权限改为  允许作为可执行文件
   `  catkin_make -DCATKIN_WHITELIST_PACKAGES="netbase_msgs"       `
   `  catkin_make -DCATKIN_WHITELIST_PACKAGES="set_nav_goals"      `
   `  catkin_make -DCATKIN_WHITELIST_PACKAGES=""      `

*__开机自启动__
   将amr_bringup中的amrStart.sh的属性改为可执行文件。

*__自动消毒__ 
  `$  sudo chmod 777 /dev/ttyUSB0 `
  `$  sudo chmod 777 /dev/ttyS1 `
  `$  roslaunch amr_bringup navigation_run.launch ` 
  `$  roslaunch amr_bringup connectPC.launch `
*__获取串口权限（每次开机后需要启动一次）__
  `$  sudo chmod 777 /dev/ttyS0 `  （串口号） //获取上位机与下位机通信串口的使用权限 
  `$  sudo chmod 777 /dev/ttyUSB0 `（串口号） //获取上位机与IMU通信串口的使用权限
  `$  dmesg | grep ttyUSB `   //查看串口使用情况，本工控机的串口号为ttyS4，IMU的串口号为ttyUSB0
  `$  sudo ls -l /dev/ttyS* 或者 sudo ls -l /dev/ttyUSB* `   //查看串口使用情况

*__手柄遥控小车__ 
  `$  roslaunch amr_bringup handle_control.launch `

*__遥控小车SLAM建图__

   修改set_slam功能包中 save_map.launch 文件中的 map_name参数 以修改建立地图的名称

  `$  roslaunch amr_bringup laser_gmapping.launch `
      
*__已知地图的自主导航（不需要遥控器）__ 
   1.修改set_navigation功能包中 nvigation.launch 文件中的 map_file参数 选择与当前环境匹配的地图
  `$  roslaunch amr_bringup navigation_run.launch `   //只启动该launch文件
   2.终止正在进行的导航任务
  `$  rostopic pub  /move_base/cancel actionlib_msgs/GoalID -- {}     `

*__连接上位机并启动任务规划__ 
  `$  roslaunch amr_bringup connectPC.launch `

*__若AMR不使用IMU__
  修改base_node中的程序，发布odom坐标变化tf,在程序中注释掉的那一行
  在建图和导航的launch文件中将hwtimu和robot_pose_ekf两个功能包的启动文件注释掉
  在set_slam/amr_gmapping.launch中修改odom_frame为base_node.cpp中发布的坐标系名称(odom),之前使用imu的时候tf由融合包发布,所以不使用imu时base_node需要发布
  在set_navigation/move_base.launch中修改odom_frame_id为base_node.cpp中发布的坐标系名称(odom)
  在set_navigation/robot_amcl.launch中修改odom_frame_id为base_node.cpp中发布的坐标系名称(odom)
  在fixed_tf_tree.launch中將imu的坐标发布去掉



*__一些基本指令__ 
  查看tf tree  
   `$  rosrun rqt_tf_tree rqt_tf_tree`  
  查看系统结构(计算图)
   `$  rqt_graph `  
  查看参数服务器  
   `$  rosparam list `  
  安装map_server
   `$  sudo apt-get install ros-kinetic-map-server `
  查看消息类型
   `$  rosmsg show nav_msgs/Odometry `
   `$  rosmsg show geometry_msgs/PoseWithCovarianceStamped `
  查看话题内容
   `$  rostopic echo /robot_pose_ekf/odom_combined `
  保存地图
   `$  rosrun map_server map_saver -f ~/AMR_ws/slam_gmapping/maps/test_map ` 
  查看两个坐标系的相对位姿	
   `$  rosrun tf tf_echo base_link base_footprint `

  终止正在进行的导航任务
  `$  rostopic pub  /move_base/cancel actionlib_msgs/GoalID -- {}    `


