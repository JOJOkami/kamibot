# kamibot

需求：通过C++和Python实现一个仿真的机器小车
1. 代通过键盘操控，使用参数设置、话题（摄像头、速度）、server(灯)
2. 代通过自动导航，可选择导航算法，SLAM建图从一个点到另一个点的避障导航

步骤：
1. 创建模型和地图
- 使用urdf语法编写一个小车模型
- 小车底盘为圆柱形
- 小车左右2轮驱动，前后2个万向轮（方便旋转）
- 小车车顶一个雷达
- 小车前方一个相机，用于实时监控并存储video

TODO：
- 通过相机匹配目标并前进到对应的目标


1. 调试驱动相关话题
2. 编写launch
3. slam算法

坑：
1、urdf和xacro的材质并不兼容，xacro还有内置的颜色
gazebo为了同时兼容rviz的urdf，加了一个标签，用于附加属性
<gazebo reference="base_link">
    <material>
    </material>
</gazebo>
基础颜色类：Gazebo/White（白色）、Gazebo/Grey（灰色）、Gazebo/DarkGrey（深灰色）、Gazebo/FlatBlack（哑光黑）、Gazebo/Black（黑色）、Gazebo/Red（红色）、Gazebo/RedBright（亮红色）、Gazebo/Green（绿色）、Gazebo/Blue（蓝色）、Gazebo/SkyBlue（天蓝色）、Gazebo/Yellow（黄色）、Gazebo/ZincYellow（锌黄色）、Gazebo/DarkYellow（深黄色）、Gazebo/Purple（紫色）、Gazebo/Turquoise（青绿色）、Gazebo/Orange（橙色）、Gazebo/Indigo（靛蓝色）。
发光效果类：Gazebo/WhiteGlow（白色发光）、Gazebo/RedGlow（红色发光）、Gazebo/GreenGlow（绿色发光）、Gazebo/BlueGlow（蓝色发光）、Gazebo/YellowGlow（黄色发光）、Gazebo/PurpleGlow（紫色发光）、Gazebo/TurquoiseGlow（青绿色发光）。

2、xmlns:xacro并不能隔绝include
通常，所有Xacro文件使用相同的xmlns:xacro声明（即http://www.ros.org/wiki/xacro）是一种最佳实践，因为这是Xacro工具的标准命名空间。
使用相同的命名空间可以确保Xacro解析器能够正确识别和处理文件中的Xacro语法。
灵活性：
理论上，你可以使用其他字符串作为命名空间，但这会导致Xacro解析器无法识别Xacro特定的标签（如<xacro:macro>），从而引发解析错误。
因此，不建议修改xmlns:xacro的值，除非你有特殊需求且明确知道如何处理解析逻辑。