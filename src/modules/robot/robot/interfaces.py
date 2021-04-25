#!/usr/bin/env python3


"""Generic ROS 2 robot launcher for interfaces such as driving, sensing and action."""


import launch
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    use_namespace = LaunchConfiguration("use_namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("params_file")
    use_odrive = LaunchConfiguration("use_odrive")

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    return launch.LaunchDescription(
        [
            DeclareLaunchArgument("namespace", description="Robot global namespace"),
            DeclareLaunchArgument(
                "use_namespace",
                default_value="true",
                description="Whether to apply a namespace to the robot stack including navigation2",
            ),
            DeclareLaunchArgument("params_file", description="Interfaces params file"),
            DeclareLaunchArgument(
                "use_sim_time", default_value="false", description="Use /clock if true"
            ),
            DeclareLaunchArgument(
                "use_odrive",
                default_value="false",
                description="Whether to launch odrive node",
            ),
            GroupAction(
                [
                    PushRosNamespace(
                        condition=IfCondition(use_namespace), namespace=namespace
                    ),
                    Node(
                        package="odrive_node",
                        executable="odrive_node",
                        output="screen",
                        parameters=[params],
                        remappings=remappings,
                    ),
                    Node(
                        package="drive",
                        executable="drive",
                        output={"both": "log"},
                        parameters=[params],
                        remappings=remappings,
                    ),
                ]
            )
            if use_odrive == "true"
            else GroupAction(
                [
                    SetEnvironmentVariable("WEBOTS_ROBOT_NAME", namespace),
                    PushRosNamespace(
                        condition=IfCondition(use_namespace), namespace=namespace
                    ),
                    Node(
                        package="drive",
                        executable="drive",
                        output="screen",
                        parameters=[params],
                        remappings=remappings,
                    ),
                ]
            ),
        ]
    )
