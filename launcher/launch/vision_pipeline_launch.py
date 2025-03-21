from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    launcher_path = FindPackageShare(
        package="image_pipeline_launcher").find("image_pipeline_launcher")
    print(launcher_path)
    params_path = os.path.join(launcher_path, "params", "config.yaml")

    color_enhancer = Node(
        package='image_pipeline_color_enhancer',
        executable='color_enhancer',
        name='color_enhancer',
        parameters=[LaunchConfiguration('param_file')]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=params_path,
            description='Path to parameter file'),
        color_enhancer
    ])
