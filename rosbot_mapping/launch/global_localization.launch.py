import os
from nav2_common.launch import ReplaceString
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation time instead of system time"
    )

    map_name_arg = DeclareLaunchArgument(
        "map_name",
        default_value="small_house",
        description="Name of the map to use"
    )

    amcl_config_arg = DeclareLaunchArgument(
        "amcl_config",
        default_value=os.path.join(
            get_package_share_directory("rosbot_mapping"),
            "config",
            "amcl.yaml"
        ),
        description="Path to the AMCL configuration file"
    )

    # Launch configurations
    map_name = LaunchConfiguration("map_name")
    use_sim_time = LaunchConfiguration("use_sim_time")
    amcl_config = LaunchConfiguration("amcl_config")

    # Define lifecycle nodes
    lifecycle_nodes = ["map_server", "amcl"]

    # Map file path
    map_path = PathJoinSubstitution([
        get_package_share_directory("rosbot_mapping"),
        "maps",
        map_name,
        "map.yaml"
    ])

    # Node: Map Server
    nav2_map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        output="screen",
        parameters=[
            {"yaml_filename": map_path},
            {"use_sim_time": use_sim_time}
        ]
    )

    # Node: AMCL
    nav2_amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        output="screen",
        parameters=[
            amcl_config,
            {"use_sim_time": use_sim_time}
        ]
    )

    # Node: Lifecycle Manager
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ]
    )

    # Return the LaunchDescription
    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        amcl_config_arg,
        nav2_map_server,
        nav2_amcl,
        nav2_lifecycle_manager
    ])
