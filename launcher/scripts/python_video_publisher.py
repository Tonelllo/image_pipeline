# Copyright(2025) UNIGE
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class VideoPublisher(Node):
    def __init__(self):
        super().__init__('video_publisher')

        # Create a publisher to send Image messages
        self.publisher = self.create_publisher(Image, 'in', 10)

        # Initialize OpenCV to read video
        self.video_capture = cv2.VideoCapture(
            '/home/tonelllo/ros2_ws/src/BetterUnderwater/media/underwater.mp4')

        # Create CvBridge to convert OpenCV images to ROS2 Image messages
        self.bridge = CvBridge()

        # Timer to publish frames at a fixed rate
        self.timer = self.create_timer(0.1, self.publish_frame)  # 10 Hz

    def publish_frame(self):
        ret, frame = self.video_capture.read()

        if ret:
            # Convert the frame from OpenCV (BGR) to ROS2 Image message (RGB)
            frame = cv2.resize(
                frame, (frame.shape[1] // 2, frame.shape[0] // 2))
            ros_image = self.bridge.cv2_to_imgmsg(frame, encoding="rgb8")

            # Publish the image
            self.publisher.publish(ros_image)

            self.get_logger().info('Publishing a frame')
        else:
            self.get_logger().info('End of video or unable to read frame.')
            # Reset video to start
            self.video_capture.set(cv2.CAP_PROP_POS_FRAMES, 0)


def main(args=None):
    rclpy.init(args=args)

    video_publisher = VideoPublisher()

    rclpy.spin(video_publisher)

    video_publisher.video_capture.release()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
