# Copyright(2025) UNIGE
import rclpy
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

        # self.img = cv2.imread(
        #     '/home/tonelllo/ros2_ws/src/image_pipeline/media/test/pipe.png')
        # self.img = cv2.imread(
        #     '/home/tonelllo/ros2_ws/src/image_pipeline/media/red_buoy.png')
        self.img = cv2.imread(
            '/home/tonelllo/ros2_ws/src/image_pipeline/media/test/buoys.png')

        # Create CvBridge to convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()

        # Timer to publish frames at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz
        self.angle = 0

    def publish_frame(self):
        out = rotate_image(self.img, self.angle)
        self.angle += 1
        ros_image = self.bridge.cv2_to_imgmsg(out, encoding="bgr8")

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
