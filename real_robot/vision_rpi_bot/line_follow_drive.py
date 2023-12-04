#!/usr/bin/env python3

import rclpy
import RPi.GPIO as GPIO
from rclpy.node import Node
from std_msgs.msg import Int16

class VisionbotDriver(Node):
    """
    Node to control the Raspberry Pi-based robot's movement based on line following error.
    """

    def __init__(self):
        super().__init__('Visionbot_driver')
        self.line_error_subscriber = self.create_subscription(Int16, '/line_following_error', self.update_motor_speed, 10)
        self.motor_controller = MotorSetup(24, 23, 25, 15, 14, 4)
        self.standard_speed = 26  # Base speed for the wheels
        self.velocity_right_wheel = 0
        self.velocity_left_wheel = 0

    def update_motor_speed(self, error_msg):
        # Adjust wheel velocities based on line following error
        if error_msg.data < 0:  # Turn right
            self.velocity_right_wheel = self.standard_speed - 10
            self.velocity_left_wheel = self.standard_speed + 10
        else:  # Turn left
            self.velocity_right_wheel = self.standard_speed + 10
            self.velocity_left_wheel = self.standard_speed - 10

        self.motor_controller.set_speeds(self.velocity_left_wheel, self.velocity_right_wheel)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = VisionbotDriver()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


class MotorSetup:
    """
    Sets up and controls the GPIO for motor operations on the Raspberry Pi robot.
    """

    def __init__(self, pin_right_a, pin_right_b, pin_right_enable, pin_left_a, pin_left_b, pin_left_enable):
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([pin_right_a, pin_right_b, pin_right_enable, pin_left_a, pin_left_b, pin_left_enable], GPIO.OUT)

        self.pwm_right = GPIO.PWM(pin_right_enable, 1000)
        self.pwm_left = GPIO.PWM(pin_left_enable, 1000)
        self.pwm_right.start(30)
        self.pwm_left.start(40)

    def set_speeds(self, left_wheel_speed, right_wheel_speed):
        # Set the speed for each wheel
        self.pwm_right.ChangeDutyCycle(right_wheel_speed)
        self.pwm_left.ChangeDutyCycle(left_wheel_speed)

    def stop_motors(self):
        # Stop the motors
        self.pwm_right.ChangeDutyCycle(0)
        self.pwm_left.ChangeDutyCycle(0)

    def cleanup(self):
        # Clean up the GPIO pins
        GPIO.cleanup()


if __name__ == '__main__':
    main()
