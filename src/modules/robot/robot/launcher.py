#!/usr/bin/env python3


"""Generic ROS 2 robot launcher."""


import os
import tempfile

import launch
from ament_index_python.packages import get_package_share_directory
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_robot_launch_description(robot_namespace: str, simulation=False):
    """Generate robot launch description based on namespace."""
    map = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    params = tempfile.NamedTemporaryFile(mode='w', delete=False)
    robot_params = os.path.join(get_package_share_directory(robot_namespace), 'param', f'{robot_namespace}.yml')
    navigation_params = os.path.join(
        get_package_share_directory('robot'), 'param', 'robot.yml')
    with open(robot_params, 'r') as r, open(navigation_params, 'r') as n:
        for f in (r, n):
            params.file.write(f.read())

    urdf = os.path.join(get_package_share_directory(robot_namespace), 'robot', f'{robot_namespace}.urdf')

    robot_launch_file_dir = os.path.dirname(__file__)
    map_dir = os.path.join(get_package_share_directory('map'), 'map', 'map.yml')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning.xml')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value=robot_namespace,
            description='Robot global namespace'
        ),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the robot stack including navigation2'
        ),

        DeclareLaunchArgument(
            'map',
            default_value=map_dir,
            description='Full path to map yaml file to load'
        ),

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use /clock if true'
        ),

        DeclareLaunchArgument(
            'bt_xml_file',
            default_value=nav2_bt_xml_file,
            description='Full path to the behavior tree xml file to use'
        ),

        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically startup the nav2 stack'
        ),


        GroupAction([

            PushRosNamespace(condition=IfCondition(
                use_namespace), namespace=namespace),

            Node(
                package='lcd_driver',
                executable='lcd_driver',
                output='screen',
                parameters=[params.name],
                remappings=remappings,
            ),

            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                arguments=[urdf],
                remappings=remappings,
            ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                output='screen',
                arguments=['0.29', '1.33', '0', '0', '0', '0', 'map', 'odom'],
                remappings=remappings,
            ),

            Node(
                package='cetautomatix',
                executable='cetautomatix',
                output='screen',
                remappings=remappings,
            ),
        ]),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([nav2_launch_file_dir, '/bringup_launch.py']),
            launch_arguments={
                'map': map,
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'bt_xml_file': bt_xml_file,
                'params_file': params.name}.items(),
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([robot_launch_file_dir, '/interfaces.py']),
            condition=IfCondition(str(not simulation)),
            launch_arguments={
                'namespace': namespace,
                'use_namespace': use_namespace,
                'use_sim_time': use_sim_time,
                'params_file': params.name}.items(),
        )
    ])
