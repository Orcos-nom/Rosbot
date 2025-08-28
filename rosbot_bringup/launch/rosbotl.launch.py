import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.conditions import UnlessCondition, IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare argument to choose SLAM or localization
    use_slam_arg = DeclareLaunchArgument(
        'use_slam', default_value='false',
        description='Whether to run SLAM or localization'
    )
    use_slam = LaunchConfiguration('use_slam')

    # Include hardware interface launch
    hardware_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbot_hardware'),
                'launch', 'rosbot_hardware.launch.py'
            )
        )
    )

    rplidar_launch = Node(
            package='rplidar_ros',
            executable='rplidar_composition',
            name='rplidar_composition',
            parameters=[{
                'serial_port': '/dev/rplidar',
                'frame_id': 'laser_link',
                'angle_compensate': True,
                'scan_mode': 'Standard'
            }],
            output='screen'
        )

    # Include localization or SLAM depending on use_slam
    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbot_mapping'),
                'launch', 'global_localization.launch.py'
            )
        ),
        condition=UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbot_mapping'),
                'launch', 'slam.launch.py'
            )
        ),
        condition=IfCondition(use_slam)
    )

    # Safety stop node
    safety_stop = Node(
        package='rosbot_utils', executable='safety_stop.py', output='screen'
    )

    return LaunchDescription([
        use_slam_arg,
        hardware_interface,
        rplidar_launch,
        localization,
        slam,
        safety_stop,
    ])
