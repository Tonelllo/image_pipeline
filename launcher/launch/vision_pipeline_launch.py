# Copyright 2025 UNIGE
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

from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node
import os


def generate_launch_description():
    launcher_path = FindPackageShare(
        package="image_pipeline_launcher").find("image_pipeline_launcher")
    params_path = os.path.join(launcher_path, "params", "config.yaml")

    color_enhancer = Node(
        package='image_pipeline_color_enhancer',
        executable='color_enhancer',
        name='color_enhancer',
        output='screen',
        parameters=[LaunchConfiguration('param_file')]
    )

    pipe_detector = Node(
        package='image_pipeline_pipe_detector',
        executable='pipe_detector',
        name='pipe_detector',
        output='screen',
        parameters=[LaunchConfiguration('param_file')]
    )

    buoy_detector = Node(
        package='image_pipeline_buoy_detector',
        executable='buoy_detector',
        name='buoy_detector',
        output='screen',
        parameters=[LaunchConfiguration('param_file')]
    )

    yolo_model = Node(
        package='image_pipeline_yolo_model',
        executable='yolo_model',
        name='yolo_model',
        output='screen',
        parameters=[
            LaunchConfiguration('param_file')
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=params_path,
            description='Path to parameter file'),
        color_enhancer,
        pipe_detector,
        buoy_detector,
        yolo_model
    ])
