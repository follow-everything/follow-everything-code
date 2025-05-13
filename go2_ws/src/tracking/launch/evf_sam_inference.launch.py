import launch
from launch import LaunchDescription
from launch_ros.actions import Node  # Ensure Node is imported
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # interactive_segmentation_node
    evf_sam_inference_node = Node(
        package='tracking',
        executable='evf_sam_inference',
        name='evf_sam_inference_node',
        parameters=[{'use_sim_time': True}],
        # output='screen'
    )


    # Return a launch description with all nodes
    return LaunchDescription([
        evf_sam_inference_node,
    ])