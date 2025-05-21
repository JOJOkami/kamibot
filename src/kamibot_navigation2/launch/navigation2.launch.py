import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    kamibot_navigation2_dir = get_package_share_directory("kamibot_navigation2")
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    rviz_config_dir = os.path.join(nav2_bringup_dir, "rviz", "nav2_default_view.rviz")
    room_file = os.path.join(kamibot_navigation2_dir, "map", "room.yaml")
    nav2_config_file = os.path.join(
        kamibot_navigation2_dir, "config", "nav2_params.yaml"
    )

    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    map_yaml_path = LaunchConfiguration("map", default=room_file)
    nav2_param_path = LaunchConfiguration("params_file", default=nav2_config_file)

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_sim_time",
                default_value=use_sim_time,
                description="Use simulation (Gazebo) clock if true",
            ),
            DeclareLaunchArgument(
                "map",
                default_value=map_yaml_path,
                description="Full path to map file to load",
            ),
            DeclareLaunchArgument(
                "params_file",
                default_value=nav2_param_path,
                description="Full path to param file to load",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [nav2_bringup_dir, "/launch", "/bringup_launch.py"]
                ),
                launch_arguments={
                    "map": map_yaml_path,
                    "use_sim_time": use_sim_time,
                    "params_file": nav2_param_path,
                }.items(),
            ),
            launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", rviz_config_dir],
                parameters=[{"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
