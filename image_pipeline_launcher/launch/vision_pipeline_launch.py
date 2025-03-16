from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    assignment_path = FindPackageShare(
        package="vision_pipeline").find("vision_pipeline")
    params_path = os.path.join(assignment_path, "params", "config.yaml")

    image_enhancer = Node(
        package='vision_pipeline',
        executable='enhancer',
        name='image_enhancer',
        parameters=[LaunchConfiguration('param_file')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file', default_value=params_path, description='Path to parameter file'),
        image_enhancer
    ])
