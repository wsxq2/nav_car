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
            "cmd_vel_topic",
            default_value="/mobile_base_controller/cmd_vel",
            description="Command velocity topic name",
        )
    )

    # Initialize Arguments
    joy_config = LaunchConfiguration("joy_config")
    joy_dev = LaunchConfiguration("joy_dev")
    cmd_vel_topic = LaunchConfiguration("cmd_vel_topic")

    # Joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
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
                [joy_config, "_config.yaml"]
            ]),
            {
                "axis_linear.x": 1,
                "axis_angular.yaw": 0, 
                "scale_linear": 0.3,
                "scale_linear_turbo": 0.6,
                "scale_angular": 0.5,
                "scale_angular_turbo": 1.0,
                "enable_button": 4,  # L1/LB
                "enable_turbo_button": 5,  # R1/RB
            }
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
