#!/usr/bin/env python3


"""Generic ROS 2 robot launcher."""


import os
import tempfile

import hiyapyco
from ament_index_python.packages import get_package_share_directory

import launch
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_robot_launch_description(robot_namespace: str, simulation=False):
    """Generate robot launch description based on namespace."""
    map = LaunchConfiguration("map")
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    default_bt_xml_filename = LaunchConfiguration("default_bt_xml_filename")
    use_odrive = LaunchConfiguration("use_odrive")

    params = tempfile.NamedTemporaryFile(mode="w", delete=False)
    robot_params = os.path.join(
        get_package_share_directory(robot_namespace), "param", f"{robot_namespace}.yml"
    )
    navigation_params = os.path.join(
        get_package_share_directory("robot"), "param", "robot.yml"
    )
    merged_params = hiyapyco.load(
        [navigation_params, robot_params], method=hiyapyco.METHOD_MERGE
    )
    params.file.write(hiyapyco.dump(merged_params))

    urdf = os.path.join(
        get_package_share_directory(robot_namespace), "robot", f"{robot_namespace}.urdf"
    )

    robot_launch_file_dir = os.path.dirname(__file__)
    nav2_bt_xml_file = os.path.join(
        get_package_share_directory("robot"),
        "behavior_trees",
        "navigate_w_replanning_time.xml",
    )

    odrive_condition = not simulation

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value=robot_namespace,
                description="Robot global namespace",
            ),
            DeclareLaunchArgument(
                "use_namespace",
                default_value="true",
                description="Whether to apply a namespace to the robot stack including navigation2",
            ),
            DeclareLaunchArgument(
                "autostart", default_value="true", description="Autostart the nav stack"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use /clock if true"
            ),
            DeclareLaunchArgument(
                "default_bt_xml_filename",
                default_value=nav2_bt_xml_file,
                description="Full path to the behavior tree xml file to use",
            ),
            DeclareLaunchArgument(
                "use_odrive",
                default_value=str(odrive_condition),
                description="Whether to launch odrive node",
            ),
            GroupAction(
                [
                    PushRosNamespace(
                        condition=IfCondition(use_namespace), namespace=namespace
                    ),
                    Node(
                        package="lcd_driver",
                        executable="lcd_driver",
                        output="screen",
                        parameters=[params.name],
                        remappings=remappings,
                    ),
                    Node(
                        package="robot_state_publisher",
                        executable="robot_state_publisher",
                        output="screen",
                        parameters=[params.name],
                        arguments=[urdf],
                        remappings=remappings,
                    ),
                    Node(
                        package="localisation",
                        executable="localisation",
                        output="screen",
                        parameters=[params.name],
                        remappings=remappings,
                    ),
                    Node(
                        package="cetautomatix",
                        executable="cetautomatix",
                        output="screen",
                        parameters=[params.name],
                        remappings=remappings,
                    ),
                    Node(
                        package="teb_obstacles",
                        executable="teb_obstacles",
                        output="screen",
                        parameters=[],
                    ),
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            [robot_launch_file_dir, "/navigation_launch.py"]
                        ),
                        launch_arguments={
                            "autostart": autostart,
                            "namespace": namespace,
                            "use_sim_time": use_sim_time,
                            "default_bt_xml_filename": default_bt_xml_filename,
                            "params_file": params.name,
                        }.items(),
                    ),
                ]
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [robot_launch_file_dir, "/interfaces.py"]
                ),
                condition=IfCondition(str(not simulation)),
                launch_arguments={
                    "namespace": namespace,
                    "use_namespace": use_namespace,
                    "use_sim_time": use_sim_time,
                    "params_file": params.name,
                    "use_odrive": use_odrive,
                }.items(),
            ),
        ]
    )
