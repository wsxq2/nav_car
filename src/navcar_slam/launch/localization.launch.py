#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
            "map_file",
            description="Full path to map yaml file to load",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "slam_config_file",
            default_value="diffbot_lds_2d_localization.lua",
            description="Cartographer localization configuration file name",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "start_rviz",
            default_value="true",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_file = LaunchConfiguration("map_file")
    slam_config_file = LaunchConfiguration("slam_config_file")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    start_rviz = LaunchConfiguration("start_rviz")

    # Include robot bringup
    robot_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("navcar_bringup"),
                "launch",
                "navcar_bringup.launch.py",
            ])
        ]),
        launch_arguments={
            "use_mock_hardware": use_mock_hardware,
            "start_rviz": "false",
        }.items(),
    )

    # Cartographer localization node
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
            "-load_state_filename",
            map_file,
        ],
        remappings=[
            ("scan", "/scan"),
            ("odom", "/odom"),
        ],
    )

    # RViz node
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("navcar_slam"), "rviz", "localization.rviz"
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
        robot_bringup,
        cartographer_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
