# kamibot仿真自动巡检机器人

## 1.项目介绍
本项目基于 ROS2 设计的一个仿真自动巡检机器人

该机器人在多个目标点进行循环移动，并统计循环路径中，出现的红点数量并记录坐标，通过摄像头采集一张图像并保存到本地

功能包功能如下：
- kamibot_base 机器人仿真相关文件，用于加载仿真机器人
- kamibot_navigation2 机器人nav2启动文件和相关配置
- kamibot_autopatrol 机器人巡航实现功能包
  - camera_node 记录打卡点
  - patrol_node 根据配置的点坐标进行自动巡航

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

## 3.目录结构
├── kamibot_autopatrol
│   ├── config              # 自动巡航功能相关配置
│   ├── launch              # 启动巡航的launch文件
│   ├── kamibot_autopatrol  # 自动巡航功能实现代码
|        ├── camera_node.py # 实现视觉匹配打卡点并拍照
|        ├── patrol_node.py # 实现指定坐标点的巡航
│   ...
├── kamibot_base
│   ├── description         # 机器人仿真模型实现代码
|        ├── sensors        # 传感器仿真模型代码
│   ├── launch              # gazebo加载仿真机器人launch文件
│   ├── world               # gazebo加载的环境文件
|   ...
└── kamibot_navigation2
│   ├── config              # Nav2配置文件
│   ├── launch              # Nav2启动launch文件
│   ├── map                 # SLAM生成的map文件用于Nav2导航
│   ...
