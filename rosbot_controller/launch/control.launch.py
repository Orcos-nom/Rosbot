import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    rosbot_controller_pkg = get_package_share_directory("rosbot_controller")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True",
        description="Use simulated time"
    )
    
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop',
        output='screen',
        prefix='xterm -e',    # ← pops up an xterm so it has a real TTY
        parameters=[ { 'stamped': True } ],
        remappings=[ ('/cmd_vel', '/diff_drive_base_controller/cmd_vel') ],
    )

    
    twist_mux_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("twist_mux")) / "launch" / "twist_mux_launch.py")
        ),
        launch_arguments={
            "cmd_vel_out": "rosbot_controller/cmd_vel_unstamped",
            "config_topics": str(Path(rosbot_controller_pkg) / "config" / "twist_mux_topics.yaml"),
            "config_lock": str(Path(rosbot_controller_pkg) / "config" / "twist_mux_locks.yaml"),
            "config_key": str(Path(rosbot_controller_pkg) / "config" / "twist_mux_key.yaml"),
            "use_sim_time": LaunchConfiguration("use_sim_time")
        }.items()
    )

    twist_relay_node = Node(
        package='rosbot_controller',
        executable='twist_relay.py',
        name='twist_mux',
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
    )

    return LaunchDescription([
        teleop_node,
        use_sim_time_arg,   
        twist_mux_launch,
        twist_relay_node, 
    ])
