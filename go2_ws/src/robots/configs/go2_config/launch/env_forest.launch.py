import os
import yaml
import xacro

import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, EnvironmentVariable
from launch.actions import SetEnvironmentVariable

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
    # Original env2_forest.launch.py configurations
    use_sim_time = LaunchConfiguration("use_sim_time")
    description_path = LaunchConfiguration("description_path")
    base_frame = "base_link"

    config_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_config"
    ).find("go2_config")
    descr_pkg_share = launch_ros.substitutions.FindPackageShare(
        package="go2_description"
    ).find("go2_description")
    
    # Configuration paths from env2_forest
    joints_config = os.path.join(config_pkg_share, "config/joints/joints.yaml")
    ros_control_config = os.path.join(
        config_pkg_share, "/config/ros_control/ros_control.yaml"
    )
    gait_config = os.path.join(config_pkg_share, "config/gait/gait.yaml")
    links_config = os.path.join(config_pkg_share, "config/links/links.yaml")
    default_model_path = os.path.join(descr_pkg_share, "xacro/robot_mid360.xacro")
    default_world_path = os.path.join(config_pkg_share, "worlds/forest.world")

    # Drone configurations
    sjtu_drone_bringup_path = get_package_share_directory('sjtu_drone_bringup')
    yaml_file_path = os.path.join(
        sjtu_drone_bringup_path,
        'config', 'drone.yaml'
    )

    # Read drone namespace from YAML
    model_ns = "drone"
    with open(yaml_file_path, 'r') as f:
        yaml_dict = yaml.load(f, Loader=yaml.FullLoader)
        model_ns = yaml_dict["namespace"]

    # Drone URDF configuration
    xacro_file_name = "sjtu_drone.urdf.xacro"
    xacro_file = os.path.join(
        get_package_share_directory("sjtu_drone_description"),
        "urdf", xacro_file_name
    )

    robot_description_config = xacro.process_file(xacro_file, mappings={"params_path": yaml_file_path})
    robot_desc = robot_description_config.toxml()

    depth_align_path = os.path.join(
        get_package_share_directory('depth_align_pkg'),
        'launch',
        'depth_align.launch.py'
    )

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )
    declare_rviz = DeclareLaunchArgument(
        "rviz", default_value="false", description="Launch rviz"
    )
    declare_robot_name = DeclareLaunchArgument(
        "robot_name", default_value="go2", description="Robot name"
    )
    declare_lite = DeclareLaunchArgument(
        "lite", default_value="false", description="Lite"
    )
    declare_ros_control_file = DeclareLaunchArgument(
        "ros_control_file",
        default_value=ros_control_config,
        description="Ros control config path",
    )
    declare_gazebo_world = DeclareLaunchArgument(
        "world", default_value=default_world_path, description="Gazebo world name"
    )
    declare_headless = DeclareLaunchArgument("headless", default_value="False")
    declare_world_init_x = DeclareLaunchArgument("world_init_x", default_value="-10")
    declare_world_init_y = DeclareLaunchArgument("world_init_y", default_value="5.0")
    declare_world_init_z = DeclareLaunchArgument("world_init_z", default_value="0.275")
    declare_world_init_heading = DeclareLaunchArgument("world_init_heading", default_value="0.0")
    declare_controller = DeclareLaunchArgument(
        "controller",
        default_value="keyboard",
        description="Type of controller: keyboard (default) or joystick",
    )

    # Include original launch files
    bringup_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_bringup"),
                "launch",
                "bringup.launch.py",
            )
        ),
        launch_arguments={
            "description_path": default_model_path,
            "joints_map_path": joints_config,
            "links_map_path": links_config,
            "gait_config_path": gait_config,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "gazebo": "true",
            "lite": LaunchConfiguration("lite"),
            "rviz": LaunchConfiguration("rviz"),
            "joint_controller_topic": "joint_group_effort_controller/joint_trajectory",
            "hardware_connected": "false",
            "publish_foot_contacts": "false",
            "close_loop_odom": "true",
        }.items(),
    )

    gazebo_ld = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "world": LaunchConfiguration("world"),
            "lite": LaunchConfiguration("lite"),
            "world_init_x": LaunchConfiguration("world_init_x"),
            "world_init_y": LaunchConfiguration("world_init_y"),
            "world_init_z": LaunchConfiguration("world_init_z"),
            "world_init_heading": LaunchConfiguration("world_init_heading"),
            "close_loop_odom": "true",
        }.items(),
    )

    depth_align_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depth_align_path)
    )

    # Drone-specific nodes
    drone_nodes = [
        # Robot State Publisher for drone
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

        # Joint State Publisher for drone
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

        # Static Transform Publisher for drone
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "world", f"{model_ns}/odom"],
            output="screen"
        ),
    ]

    return LaunchDescription(
        [
            # Original launch arguments
            declare_use_sim_time,
            declare_rviz,
            declare_robot_name,
            declare_lite,
            declare_ros_control_file,
            declare_gazebo_world,
            declare_headless,
            declare_world_init_x,
            declare_world_init_y,
            declare_world_init_z,
            declare_world_init_heading,
            declare_controller,
            
            # Original launch descriptions
            bringup_ld,
            gazebo_ld,
            depth_align_launch,
            
            # Drone-specific nodes
            *drone_nodes,
            
            # Teleop controller
            OpaqueFunction(
                function=get_teleop_controller,
                kwargs={'model_ns': model_ns},
            ),
        ]
    )