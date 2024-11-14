import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Paths to package directories if needed for configurations or parameters
    package_dir = get_package_share_directory('pallet_processing')
    semseg_package_dir = get_package_share_directory('semseg')

    # Launch configuration for setting node parameters (if any)
    return LaunchDescription([
        # Declare launch arguments if needed
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'
        ),

        # Node 1: Initial Data Processing Node
        Node(
            package='pallet_processing',
            executable='initial_data_processing_node',
            name='initial_data_processing_node',
            output='screen'
        ),

        # Node 2: YOLO Detection Node
        Node(
            package='pallet_processing',
            executable='yolo_detection_node',
            name='yolo_detection_node',
            output='screen'
        ),

        # Node 3: Segmentation Node
        Node(
            package='pallet_processing',
            executable='segment_node',
            name='segment_node',
            output='screen'
        ),

        # Node 4: Dummy Image Publisher Node (if needed)
        Node(
            package='pallet_processing',
            executable='dummy_image_node',
            name='dummy_image_node',
            output='screen'
        ),

        # Add other nodes similarly if needed
    ])
