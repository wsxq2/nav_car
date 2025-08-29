#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulation/Gazebo clock if true",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam_config_file",
            default_value="diffbot_lds_2d.lua",
            description="Cartographer configuration file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config_file = LaunchConfiguration("slam_config_file")
    start_rviz = LaunchConfiguration("start_rviz")

    # Cartographer SLAM node
    cartographer_node = Node(
        package="cartographer_ros",
        executable="cartographer_node",
        name="cartographer_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=[
            "-configuration_directory",
            PathJoinSubstitution([FindPackageShare("navcar_slam"), "config"]),
            "-configuration_basename",
            slam_config_file,
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/mobile_base_controller/odom"),
        ],
    )

    # Cartographer occupancy grid node
    occupancy_grid_node = Node(
        package="cartographer_ros",
        executable="cartographer_occupancy_grid_node",
        name="occupancy_grid_node",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        arguments=["-resolution", "0.05", "-publish_period_sec", "1.0"],
    )

    # RViz node with SLAM configuration
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("navcar_slam"), "rviz", "slam.rviz"
    ])
    
    rviz_node = Node(
        condition=IfCondition(start_rviz),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    nodes = [
        cartographer_node,
        occupancy_grid_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
