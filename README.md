# kamibot仿真自动巡检机器人

## 1.项目介绍
本项目基于 ROS2 设计的一个仿真自动巡检机器人

该机器人在多个目标点进行循环移动，并统计循环路径中，出现的红点数量并记录坐标，通过摄像头采集一张图像并保存到本地

功能包功能如下：
- kamibot_base 机器人仿真相关文件，用于加载仿真机器人
- kamibot_application 机器人导航代码
- kamibot_navigation2 机器人nav2启动文件和相关配置
- kamibot_autopatrol 机器人巡航实现功能包

## 2.使用方法

平台信息：
- 系统版本：Ubuntu 22.04.5
- ROS 版本：ROS2 humble

### 2.1 安装

1、安装slam-toolbox用于建图
```shell
sudo apt install ros-$ROS_DISTRO-slam-toolbox
```

2、安装Navgation2用于自动巡航
```shell
sudo apt install ros-$ROS_DISTRO-nav2-bringup
```

3、安装Gazebo仿真相关功能包
```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher  ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro
```

### 2.2运行

安装完成依赖后，可以使用 colcon 工具进行构建和运行。

1、构建功能包

```shell
colcon build
```

2、运行仿真机器人
```shell
source install/setup.bash
ros2 launch kamibot_base load_robot_into_gazebo.launch.py
```

3、运行navgation2
```shell
source install/setup.bash
ros2 launch kamibot_navigation2  navigation2.launch.py
```

4、运行巡航程序
```shell
source install/setup.bash
ros2 launch kamibot_autopatrol autopatrol.launch.py
```