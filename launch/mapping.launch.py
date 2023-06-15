import os.path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    package_path = get_package_share_directory("fast_lio")
    default_config_path = os.path.join(package_path, "config", "velodyne.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    config_path = LaunchConfiguration("config_path")

    arg_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )
    arg_config_path = DeclareLaunchArgument(
        "config_path",
        default_value=default_config_path,
        description="Yaml config file path",
    )
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        parameters=[config_path, {"use_sim_time": use_sim_time}],
        output="screen",
        emulate_tty=True,
    )
    ld = LaunchDescription()
    ld.add_action(arg_use_sim_time)
    ld.add_action(arg_config_path)
    ld.add_action(fast_lio_node)
    # ld.add_action(rviz_node)

    return ld
