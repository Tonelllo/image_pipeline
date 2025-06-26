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

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os


def generate_launch_description():
    launcher_path = get_package_share_directory("image_pipeline_launcher")
    params_path = os.path.join(launcher_path, "params", "config.yaml")
    param_file = LaunchConfiguration('param_file')

    color_enhancer_component = ComposableNode(
        package='image_pipeline_color_enhancer',
        plugin='image_pipeline::ColorEnhancer',
        name='color_enhancer',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[param_file]
    )

    pipe_detector_component = ComposableNode(
        package='image_pipeline_pipe_detector',
        plugin='image_pipeline::PipeDetector',
        name='pipe_detector',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[param_file],
    )

    buoy_detector_component = ComposableNode(
        package='image_pipeline_buoy_detector',
        plugin='image_pipeline::BuoyDetector',
        name='buoy_detector',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[param_file]
    )

    yolo_model_component = ComposableNode(
        package='image_pipeline_yolo_model',
        plugin='image_pipeline::YoloModel',
        name='yolo_model',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[
            param_file
        ],
    )

    buoy_color_component = ComposableNode(
        package='image_pipeline_buoy_color',
        plugin='image_pipeline::BuoyColor',
        name='buoy_color',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[
            param_file
        ],
    )

    camera_info_publisher = ComposableNode(
        package='image_pipeline_camera_info_publisher',
        plugin='image_pipeline::CameraInfoPublisher',
        name='camera_info_publisher',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[
            param_file
        ],
    )

    watchdog = ComposableNode(
        package='image_pipeline_watchdog',
        plugin='image_pipeline::WatchDog',
        name='watchdog',
        namespace='image_pipeline',
        extra_arguments=[{'use_intra_process_comms': True}],
        parameters=[
            param_file
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'param_file',
            default_value=params_path,
            description='Path to parameter file'),
        ComposableNodeContainer(
            name='main_container',
            namespace='image_pipeline',
            package='rclcpp_components',
            executable='component_container',
            # prefix='kitty -e gdb --args',
            composable_node_descriptions=[
                color_enhancer_component,
                # pipe_detector_component,
                # buoy_detector_component,
                yolo_model_component,
                buoy_color_component,
                camera_info_publisher,
                watchdog
            ],
            output='screen'
        )
    ])
