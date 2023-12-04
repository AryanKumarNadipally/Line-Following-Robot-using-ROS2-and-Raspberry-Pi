# ROS2 Python Node for Publishing Video Stream from Raspberry Pi Camera
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class RPiCameraPublisher(Node):
    """
    This node captures video from the Raspberry Pi camera and publishes it as ROS2 Image messages.
    """

    def __init__(self):
        super().__init__('rpi_camera_publisher')
        self.image_publisher = self.create_publisher(Image, '/rpi_video_feed', 10)
        publish_rate = 0.05  # Publishing rate in seconds
        self.publish_timer = self.create_timer(publish_rate, self.publish_camera_frame)
        self.camera = cv2.VideoCapture(0)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.image_converter = CvBridge()

    def publish_camera_frame(self):
        """
        Captures a frame from the camera, converts it to a ROS2 Image message, and publishes it.
        """
        success, frame = self.camera.read()
        if not success:
            return  # Skip the frame if capture failed

        frame_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ros_image = self.image_converter.cv2_to_imgmsg(frame_gray, 'mono8')
        self.image_publisher.publish(ros_image)


def main(args=None):
    rclpy.init(args=args)

    video_stream_publisher = RPiCameraPublisher()
    print("Raspberry Pi Camera Node Started")
    rclpy.spin(video_stream_publisher)
    video_stream_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
