#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_config",
            default_value="ps3",
            description="Joystick configuration to use (ps3 or xbox)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev", 
            default_value="/dev/input/js0",
            description="Joystick device path",
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
            default_value="false",
            description="Start RViz2 automatically with this launch file.",
        )
    )

    # Initialize Arguments
    joy_config = LaunchConfiguration("joy_config")
    joy_dev = LaunchConfiguration("joy_dev")
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
            "start_rviz": start_rviz,
        }.items(),
    )

    # Include joystick control
    joystick_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("navcar_joytick"),
                "launch", 
                "navcar_joystick.launch.py",
            ])
        ]),
        launch_arguments={
            "joy_config": joy_config,
            "joy_dev": joy_dev,
            "cmd_vel_topic": "/mobile_base_controller/cmd_vel",
        }.items(),
    )

    return LaunchDescription(declared_arguments + [
        robot_bringup,
        joystick_control,
    ])
