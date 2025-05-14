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

import rclpy
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


def rotate_image(image, angle):
    image_center = tuple(np.array(image.shape[1::-1]) / 2)
    rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
    result = cv2.warpAffine(
        image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
    return result


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher', namespace='image_pipeline')

        self.publisher = self.create_publisher(Image, 'fake_video', 10)

        self.img = cv2.imread(
            '/home/tonelllo/ros2_ws/src/image_pipeline/media/test/buoys.png')
        # self.img = cv2.imread(
        #     '/home/tonelllo/ros2_ws/src/image_pipeline/media/red_buoy.png')
        # self.img = cv2.imread(
        #     '/home/tonelllo/ros2_ws/src/image_pipeline/media/test/buoys.png')
        # self.img = cv2.imread(
        #     '/home/tonelllo/ros2_ws/src/image_pipeline/media/red_buoy.png')
        self.img = cv2.imread(
            '/home/tonelllo/ros2_ws/src/image_pipeline/media/test/pipe.png')

        # Create CvBridge to convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()

        # Timer to publish frames at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz
        self.angle = 0

    def publish_frame(self):
        out = rotate_image(self.img, self.angle)
        self.angle += 1
        height = 384
        width = 640
        start_time = time.time()
        resized_image = cv2.resize(out, (width, height))
        end_time = time.time()
        print("Resize took: ", end_time - start_time)
        ros_image = self.bridge.cv2_to_imgmsg(resized_image, encoding="bgr8")

        # Publish the image
        self.publisher.publish(ros_image)

        self.get_logger().info('Publishing a frame')


def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    rclpy.spin(video_publisher)

    video_publisher.video_capture.release()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
