import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class LineFollower(Node):
    def __init__(self):
        super().__init__('qr_maze_solving_node')
        self.subscriber = self.create_subscription(Image, '/vision_rpi_bot_camera/image_raw', self.process_image, 10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.movement_command = Twist()
        self.image_converter = CvBridge()

    def process_image(self, img_data):
        cv_image = self.image_converter.imgmsg_to_cv2(img_data, 'bgr8')
        cropped_image = cv_image[290:479, 130:400]
        edge_detected = cv2.Canny(cropped_image, 60, 100)

        white_pixels = []
        center_of_line = 0
        for idx, pixel in enumerate(edge_detected[:][172]):
            if pixel == 255:
                white_pixels.append(idx)

        if len(white_pixels) == 2:
            for point in white_pixels:
                cv2.circle(edge_detected, (point, 172), 2, (255, 0, 0), 1)
            center_of_line = int(sum(white_pixels) / 2)
            cv2.circle(edge_detected, (center_of_line, 172), 3, (255, 0, 0), 2)

        robot_center = [135, 172]
        cv2.circle(edge_detected, tuple(robot_center), 5, (255, 0, 0), 2)
        line_error = robot_center[0] - center_of_line

        self.adjust_robot_movement(line_error)
        self.display_images(cv_image, edge_detected)

    def adjust_robot_movement(self, error):
        angular_direction = -0.5 if error < 0 else 0.5
        self.movement_command.angular.z = angular_direction
        self.movement_command.linear.x = 0.4
        self.velocity_publisher.publish(self.movement_command)

    def display_images(self, original, edge):
        cv2.imshow('Original Frame', original)
        cv2.imshow('Edge Detection', edge)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    line_follower_node = LineFollower()
    rclpy.spin(line_follower_node)
    line_follower_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
