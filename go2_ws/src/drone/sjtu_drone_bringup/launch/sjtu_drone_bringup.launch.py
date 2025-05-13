#!/usr/bin/env python3
# Copyright 2023 Georg Novotny
#
# Licensed under the GNU GENERAL PUBLIC LICENSE, Version 3.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.gnu.org/licenses/gpl-3.0.en.html
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import yaml
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    OpaqueFunction
)
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def get_teleop_controller(context, *_, **kwargs) -> Node:
    """
    Select and configure the teleop controller based on launch configuration
    """
    controller = context.launch_configurations["controller"]
    namespace = kwargs["model_ns"]

    if controller == "joystick":
        node = Node(
            package="sjtu_drone_control",
            executable="teleop_joystick",
            namespace=namespace,
            output="screen",
        )
    else:
        node = Node(
            package="sjtu_drone_control",
            executable="teleop",
            namespace=namespace,
            output="screen",
            prefix="xterm -e",
        )

    return [node]


def generate_launch_description():
    # Paths and configurations
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # YAML configuration
    yaml_file_path = os.path.join(
        sjtu_drone_bringup_path,
        'config', 'drone.yaml'
    )

    # Read namespace from YAML
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    # URDF configuration
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )

    # Process URDF
    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()

    # World file configuration
    world_file_default = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "worlds", "playground.world"
    )

    # Launch configuration variables
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")
    world_file = LaunchConfiguration('world', default=world_file_default)

    return LaunchDescription([
        # Launch Arguments
        DeclareLaunchArgument(
            "controller",
            default_value="keyboard",
            description="Type of controller: keyboard (default) or joystick",
        ),
        DeclareLaunchArgument(
            "use_gui", 
            default_value="true", 
            choices=["true", "false"],
            description="Whether to execute gzclient"
        ),
        DeclareLaunchArgument(
            name='world',
            default_value=world_file_default,
            description='Full path to world file to load'
        ),

        # Gazebo Server
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={
                'world': world_file,
                'verbose': "true",
                'extra_gazebo_args': 'verbose'
            }.items()
        ),

        # Gazebo Client (Conditional)
        OpaqueFunction(
            function=lambda context, *args, **kwargs: 
                [IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                    ),
                    launch_arguments={'verbose': 'true'}.items()
                )] if context.launch_configurations.get('use_gui') == 'true' else []
        ),

        # Robot State Publisher
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            namespace=model_ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time, 
                "robot_description": robot_desc, 
                "frame_prefix": model_ns + "/"
            }],
            arguments=[robot_desc]
        ),

        # Joint State Publisher
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            namespace=model_ns,
            output='screen',
        ),

        # Joystick Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy',
            namespace=model_ns,
            output='screen',
        ),

        # Spawn Drone
        Node(
            package="sjtu_drone_bringup",
            executable="spawn_drone",
            arguments=[robot_desc, model_ns],
            output="screen"
        ),

        # Static Transform Publisher
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),

        # Teleop Controller
        OpaqueFunction(
            function=get_teleop_controller,
            kwargs={'model_ns': model_ns},
        ),
    ])