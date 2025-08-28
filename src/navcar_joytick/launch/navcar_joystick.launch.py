#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "joy_dev", 
            default_value="/dev/input/js0",
            description="Joystick device path",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "cmd_vel_topic",
            default_value="/mobile_base_controller/cmd_vel",
            description="Command velocity topic name",
        )
    )

    # Initialize Arguments
    joy_dev = LaunchConfiguration("joy_dev")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    # Joy node
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joystick",
        parameters=[
            {"device_name": joy_dev},
            {"deadzone": 0.1},
            {"autorepeat_rate": 10.0},
        ],
    )

    # Teleop twist joy node  
    teleop_node = Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        name="teleop_twist_joy_node",
        parameters=[
            PathJoinSubstitution([
                FindPackageShare("navcar_joytick"),
                "config",
                "ps3_config.yaml"
            ])
        ],
        remappings=[
            ("cmd_vel", cmd_vel_topic),
        ],
    )

    nodes = [
        joy_node,
        teleop_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
