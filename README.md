[TOC]

# 实验要求

```
使用装有尖刺的 Parrot Bebop 无人机在 100X60m 的区域内寻找红色气球并刺破
```

# 配置要求

```
硬件配置：笔记本电脑；Bebop四旋翼无人机

系统环境：Ubuntu16.04

软件工具：ROS(kinetic 版本)、parrot2、yolov3（[ROS](https://github.com/ShiSanChuan/darknet_ros/blob/master/www.ros.org)是一款为机器人设计的系统框架，[parrot2](http://www.parrot.com.cn/)无人机提供ROS的开发通信环境[bebop_autonomy](http://wiki.ros.org/bebop_autonomy)、[darknet yolov3](https://pjreddie.com/darknet/yolo/)是深度学习的目标检测框架）
```

# 环境配置

```
Ubuntu16.04下安装ROS Kinetic: <https://blog.csdn.net/wangguchao/article/details/81044558>

安装依赖包: 终端中输入 sudo apt-get install build-essential python-rosdep python-catkin-tools
```

# 项目地址

<https://github.com/wuxiangfei0011/drone-pricked-red-ball>

# 操作流程

```
拷贝无人机代码 bebop_ws 到 home 目录下

PC 端连接 parrot2 发出的 wifi：

终端中依次输入（按下 Ctrl+Alt+T 打开终端）
```

```
gedit .bashrc
在打开的文件最后一行输入：source ~/bebop_ws/devel/setup.bash
关闭文件
在终端中输入：source ~/.bashrc

关闭终端
打开新终端窗口（使得上行的 source 生效）
终端中输入（登录无人机）：roslaunch bebop_tools bebop_nodelet_iv.launch

打开新终端窗口
终端中输入（开启摄像头识别气球）：rosrun bebop_driver img_trans.py

打开新终端窗口
终端中输入：rostopic pub -1 bebop/takeoff std_msgs/Empty 起飞开始识别扎气球

停止无人机扎球
在以上所有终端中按下按键：Ctrl+C

打开新终端窗口
无人机降落：rostopic pub -1 bebop/land std_msgs/Empty
```

# 无人机其他控制命令

```
Parrot bebop2 运行命令
roslaunch bebop_driver bebop_node.launch 登录
roslaunch bebop_tools bebop_nodelet_iv.launch
rostopic pub -1 bebop/reset std_msgs/Empty  紧急停止
rostopic pub -1 bebop/land std_msgs/Empty   降落 
rostopic pub -1 bebop/takeoff std_msgs/Empty 起飞
rostopic pub -1 bebop/cmd_vel geometry_msgs/Twist --'[2.0,0.0,0.0]' '[0.0,0.0,1.8]' 以z轴为中心，每秒旋转1.8rad
source ~/bebop_ws/devel/setup.bash 启动工作空间
roslaunch bebop_driver bebop_node.launch 节点启动
rosrun bebop_driver teleop_twist_keyboard.py 键盘启动

Turtlebot3 运行命令
  
source ~/turtlebot3/devel/setup.bash 启动工作空间
export TURTLEBOT3_MODEL=burger 环境变量设置

roslaunch turtlebot3_fake turtlebot3_fake.launch 启动仿真界面

炸气球命令

source ~/catch_ball/devel/setup.bash
```

# 参考
* Documentation: <http://bebop-autonomy.readthedocs.io/>
* ROS wiki page: <http://wiki.ros.org/bebop_autonomy>
* Support: [ROS Q&A (tag: bebop_autonomy)](http://answers.ros.org/questions/scope:all/sort:activity-desc/tags:bebop_autonomy/page:1/)
* Code API: <http://docs.ros.org/indigo/api/bebop_autonomy/html>
* ROS与Bebop入门教程: <https://www.ncnynl.com/category/ros-bebop/>
* <https://blog.csdn.net/u011591807/article/details/106219893/>