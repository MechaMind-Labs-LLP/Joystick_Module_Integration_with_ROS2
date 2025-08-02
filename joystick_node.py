#!/usr/bin/env python3

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class JoystickSerialToTurtle(Node):
    def __init__(self):
        super().__init__("joystick_serial_to_turtle")

        # Parameters
        self.declare_parameter("port", "/dev/ttyACM0")
        self.declare_parameter("baudrate", 115200)

        self.port_ = self.get_parameter("port").value
        self.baudrate_ = self.get_parameter("baudrate").value

        # Serial connection
        try:
            self.arduino_ = serial.Serial(port=self.port_, baudrate=self.baudrate_, timeout=0.1)
            self.get_logger().info(f"Connected to Arduino on {self.port_}")
        except serial.SerialException:
            self.get_logger().error(f"Could not connect to Arduino on {self.port_}")
            exit(1)

        # Publisher
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

        # Timer
        self.timer_ = self.create_timer(0.05, self.timerCallback)  # 20Hz

    def timerCallback(self):
        if rclpy.ok() and self.arduino_.is_open:
            try:
                line = self.arduino_.readline().decode("utf-8").strip()
                if not line:
                    return

                parts = line.split(',')
                if len(parts) != 2:
                    return

                x_val, y_val = map(int, parts)

                # Normalize and apply deadzone
                linear = (y_val - 512) / 512.0
                angular = -(x_val - 512) / 512.0

                deadzone = 0.1
                if abs(linear) < deadzone:
                    linear = 0.0
                if abs(angular) < deadzone:
                    angular = 0.0

                # Publish to turtle
                twist = Twist()
                twist.linear.x = linear*3.0
                twist.angular.z = angular*2.0
                self.cmd_vel_pub_.publish(twist)

            except Exception as e:
                self.get_logger().error(f"Serial read failed: {e}")


def main():
    rclpy.init()
    node = JoystickSerialToTurtle()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
