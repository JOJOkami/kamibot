import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory("kamibot_autopatrol"))
    patrol_params_file = os.path.join(pkg_path, "config", "patrol_config.yaml")
    camera_params_file = os.path.join(pkg_path, "config", "camera_config.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    launch_config_camera = LaunchConfiguration("params_file", default=camera_params_file)
    launch_config_patrol = LaunchConfiguration("params_file", default=patrol_params_file)

    # 发布巡航节点
    camera_node = Node(
        package="kamibot_autopatrol",
        executable="camera_node",
        output="screen",
        parameters=[launch_config_camera, use_sim_time],
    )

    # 发布相机节点
    patrol_node = Node(
        package="kamibot_autopatrol",
        executable="patrol_node",
        output="screen",
        parameters=[launch_config_patrol, use_sim_time],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="true",
                description="Use simulation (Gazebo) clock if true",
            ),
            patrol_node,
            camera_node,
        ]
    )
