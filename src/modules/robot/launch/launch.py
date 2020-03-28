#!/usr/bin/env python3


"""Generic ROS 2 robot launcher."""


import os
import launch
import tempfile
from launch.conditions import IfCondition
from launch.actions import GroupAction
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    map = LaunchConfiguration('map')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    bt_xml_file = LaunchConfiguration('bt_xml_file')

    # Parameters fusion
    robot_params = os.path.join(get_package_share_directory(namespace), 'param', f'{namespace}.yml')
    navigation_params = os.path.join(get_package_share_directory('robot'), 'param', 'robot.yml')
    params = tempfile.NamedTemporaryFile(mode='w', delete=False)
    with open(robot_params, 'r') as r, open(navigation_params, 'r') as n:
        for f in (r, n):
            params.file.write(f.read())

    map_dir = os.path.join(get_package_share_directory('map'), 'map', 'map.yml')
    urdf = os.path.join(get_package_share_directory(namespace), 'robot', f'{namespace}.urdf')
    nav2_launch_file_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')
    nav2_bt_xml_file = os.path.join(get_package_share_directory('nav2_bt_navigator'), 'behavior_trees', 'navigate_w_replanning.xml')

    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]

    return launch.LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='robot',
            description='Robot global namespace'
        ),

        DeclareLaunchArgument(
            'use_namespace',
            default_value='true',
            description='Whether to apply a namespace to the robot stack including navigation2'
        ),

        DeclareLaunchArgument(
            'map',
            default=map_dir,
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
            PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace),

            Node(
                package='lcd_driver',
                node_executable='lcd_driver',
                output='screen',
                parameters=[params],
                remappings=remappings,
            ),

            Node(
                package='drive',
                node_executable='drive',
                output='screen',
                parameters=[params],
                remappings=remappings,
            ),

            Node(
                package='robot_state_publisher',
                node_executable='robot_state_publisher',
                output='screen',
                arguments=[urdf],
                remappings=remappings,
            ),

            Node(
                package='tf2_ros',
                node_executable='static_transform_publisher',
                output='screen',
                arguments=['0.15', '1.0', '0', '0', '0', '0', 'map', 'odom'],
                remappings=remappings,
            ),

            Node(
                package='cetautomatix',
                node_executable='cetautomatix',
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
                'params_file': params}.items(),
        )
    ])
