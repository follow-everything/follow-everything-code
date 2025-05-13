import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Ensure Node is imported
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # interactive_segmentation_node
    interactive_segmentation_node = Node(
        package='tracking',
        executable='interactive_segmentation',
        name='interactive_segmentation_node',
        parameters=[{'use_sim_time': True}],
        # output='screen'
    )

    # tracking_difference_node
    # tracking_difference_node = Node(
    #     package='tracking',
    #     executable='tracking_difference',
    #     name='tracking_difference_node',
    #     parameters=[{'use_sim_time': True}],
    #     output='screen'
    # )

    # filter_cloudpoint node
    # filter_cloudpoint_node = Node(
    #     package='tracking',
    #     executable='filter_cloudpoint',  # Matches the entry point in setup.py
    #     name='filter_cloudpoint_node',
    #     output='screen'
    # )

    # Return a launch description with all nodes
    return LaunchDescription([
        interactive_segmentation_node,
        # tracking_difference_node,
        # filter_cloudpoint_node  # Include the new node
    ])