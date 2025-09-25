#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the launch directory
    pkg_navcar_navigation = get_package_share_directory('navcar_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    # Declare launch arguments
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    default_nav_to_pose_bt_xml = LaunchConfiguration('default_nav_to_pose_bt_xml')
    default_nav_through_poses_bt_xml = LaunchConfiguration('default_nav_through_poses_bt_xml')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_navcar_navigation, 'config', 'nav2_params.yaml'),
        description='Full path to param file to load')

    declare_bt_xml_cmd = DeclareLaunchArgument(
        'default_nav_to_pose_bt_xml',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_to_pose_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    declare_bt_xml_cmd2 = DeclareLaunchArgument(
        'default_nav_through_poses_bt_xml',
        default_value=os.path.join(
            get_package_share_directory('nav2_bt_navigator'),
            'behavior_trees', 'navigate_through_poses_w_replanning_and_recovery.xml'),
        description='Full path to the behavior tree xml file to use')

    # Nav2 navigation launch (without map server and AMCL)
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'params_file': params_file,
            'default_nav_to_pose_bt_xml': default_nav_to_pose_bt_xml,
            'default_nav_through_poses_bt_xml': default_nav_through_poses_bt_xml,
        }.items()
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_bt_xml_cmd)
    ld.add_action(declare_bt_xml_cmd2)

    # Add nav2 navigation
    ld.add_action(nav2_navigation)

    return ld
