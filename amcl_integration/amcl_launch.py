import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    this_dir = os.path.dirname(os.path.abspath(__file__))
    default_amcl_params = os.path.join(this_dir, "amcl_params.yaml")

    declare_map_file = DeclareLaunchArgument(
        "map",
        description="Full path to the map yaml file to load"
    )

    declare_params_file = DeclareLaunchArgument(
        "params_file",
        default_value=default_amcl_params,
        description="Full path to the AMCL parameters yaml file"
    )

    # Map Server Node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[{
            "yaml_filename": LaunchConfiguration("map"),
            "use_sim_time": False,
        }],
        output="screen",
    )

    # AMCL Node
    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[LaunchConfiguration("params_file")],
        output="screen",
    )

    # Lifecycle Manager for Localization (starts map_server & amcl)
    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
        }],
        output="screen",
    )

    return LaunchDescription([
        declare_map_file,
        declare_params_file,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
