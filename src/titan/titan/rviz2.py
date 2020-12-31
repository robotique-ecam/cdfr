#!/usr/bin/env python3
# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from titan.utils.replace_string import ReplaceString


def generate_rviz_launch_description(namespace="robot"):
    # Get the launch directory
    titan_dir = get_package_share_directory("titan")

    rviz_config_file = os.path.join(titan_dir, "rviz", "namespaced_view.rviz")

    use_namespace = "true"

    namespaced_rviz_config_file = ReplaceString(
        source_file=rviz_config_file,
        replacements={"<robot_namespace>": ("/", namespace)},
    )

    start_namespaced_rviz_cmd = Node(
        condition=IfCondition(use_namespace),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        namespace=namespace,
        # TODO: FIX onload MUTEX ERROR
        # arguments=['-d', namespaced_rviz_config_file],
        output="screen",
        remappings=[
            ("/tf", "tf"),
            ("/tf_static", "tf_static"),
            ("/goal_pose", "goal_pose"),
            ("/clicked_point", "clicked_point"),
            ("/initialpose", "initialpose"),
        ],
    )

    exit_event_handler_namespaced = RegisterEventHandler(
        condition=IfCondition(use_namespace),
        event_handler=OnProcessExit(
            target_action=start_namespaced_rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason="rviz exited")),
        ),
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add any conditioned actions
    ld.add_action(start_namespaced_rviz_cmd)

    # Add other nodes and processes we need
    ld.add_action(exit_event_handler_namespaced)

    return ld
