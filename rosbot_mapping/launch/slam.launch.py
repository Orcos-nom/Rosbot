import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",  
        description="Use simulation time"
    )
    
    slam_config_path = os.path.join(
        get_package_share_directory("rosbot_mapping"),
        "config",
        "slam_toolbox.yaml"
    )
    
    if not os.path.exists(slam_config_path):
        print(f"[ERROR] SLAM config file not found: {slam_config_path}")
    
    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=slam_config_path,
        description="Full path to the SLAM toolbox config file"
    )
    
    slam_config = LaunchConfiguration("slam_config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    
    lifecycle_nodes = ["slam_toolbox", "map_saver_server"]
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
            {"map_frame": "map"},
            {"odom_frame": "odom"},
            {"base_frame": "base_footprint"},
        ]
    )
    
    # Map Saver Node
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65}
        ]
    )

    # Lifecycle Manager for SLAM Toolbox
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )
    
    return LaunchDescription([
        LogInfo(msg="Starting SLAM Toolbox Launch"),
        use_sim_time_arg,
        slam_config_arg,
        slam_toolbox_node,
        nav2_map_saver,
        nav2_lifecycle_manager
    ])
