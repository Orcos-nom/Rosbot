import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition , IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    use_slam_arg = DeclareLaunchArgument(
        "use_slam",
        default_value="false"
    )

    use_slam = LaunchConfiguration("use_slam")


    # Gazebo launch file
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbot_description"),
                "launch",
                "sp_gz.launch.py"
            )
        )
    )

    twist_mux_params = os.path.join(get_package_share_directory("rosbot_controller"),'config','twist_mux_topics.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    localization = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "launch",
            "global_localization.launch.py"
        ),
        condition = UnlessCondition(use_slam)
    )

    slam = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "launch",
            "slam.launch.py"
        ),
        condition = IfCondition(use_slam)
    )

    safety_stop = Node(
       package="rosbot_utils",
       executable="safety_stop.py",
       output="screen"
    )

    rviz_locatization = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d",os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "rviz",
            "global_localization.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time":True}],
        condition=UnlessCondition(use_slam)
    )

    rviz_slam = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d",os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "rviz",
            "slam.rviz"
        )],
        output="screen",
        parameters=[{"use_sim_time":True}],
        condition=IfCondition(use_slam)
    )

    return LaunchDescription([
        use_slam_arg,
        gazebo,
        twist_mux,
        safety_stop,
        localization,
        slam,
        rviz_locatization,
        rviz_slam
    ])
