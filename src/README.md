
#使用方法
 将三个功能包(dg_common,joy_control,third_libs)放在src路径下面
 可在joy_control修改发布话题的名称，默认为 cmd_vel
*__启动launch文件__
` roslaunch joy_control we_joy_control.launch `

#编译报错时执行：
* __错误1__
1.Could not find a package configuration file provided by "joy" with any of
  the following names:
    joyConfig.cmake
    joy-config.cmake

` $ sudo apt-get install ros-kinetic-joy* `


* __错误2__
2.程序编译时候出现了 找不到 histedit.h: 头文件的情况:

` sudo apt-get install libedit-dev  `


* __错误3__
3.error while loading shared libraries: libboost_thread.so.1.54.0: cannot open shared object file: No such file or directory

查看有无libboost_thread.so.1.58.0
` locate libboost_thread.so.1.58.0 `
若有建立软连接libboost_system
` sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_system.so.1.58.0  /usr/lib/x86_64-linux-gnu/libboost_system.so.1.54.0 `
若有建立软连接libboost_thread
` sudo ln -s /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.58.0  /usr/lib/x86_64-linux-gnu/libboost_thread.so.1.54.0  `

` https://blog.csdn.net/qq_36501182/article/details/79516381  `

* __错误4__
4.[joy_teleop-2] process has died [pid 416, exit code -11, cmd /home/bianjingyang/exercise_ws/devel/lib/joy_control/joy_teleop __name:=joy_teleop __log:=/home/bianjingyang/.ros/log/6817d820-be8c-11ea-8eca-84fdd1f0faa0/joy_teleop-2.log].

` 修改we_joy_control.launch文件中的结点注释块，笔记本电脑将第一个和第四个启动节点注释掉 `

`本质原因是要修改 手柄接收器的通讯接口号，we_joy_control.yaml中的 dev: "/dev/input/js0"  参数`
`若出现错误4,尝试将dev: "/dev/input/js0"改为dev: "/dev/input/js1" `


