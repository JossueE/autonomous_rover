#!/usr/bin/env python3

"""
Node to move a differential drive mobile robot using a game controller.
Reads from /joy topic and publishes to cmd_vel_safe.
Includes a deadman switch on the RT trigger.
"""

__author__ = "Jules"

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class TeleopJoycon(Node):
    def __init__(self):
        super().__init__('teleop_joycon')

        self.declare_parameter('max_lin_vel', 0.7)
        self.declare_parameter('max_ang_vel', 0.8)
        self.declare_parameter('axis_linear', 1)   # Left joystick Up/Down
        self.declare_parameter('axis_angular', 0)  # Left joystick Left/Right

        self.max_lin_vel = self.get_parameter('max_lin_vel').value
        self.max_ang_vel = self.get_parameter('max_ang_vel').value
        self.axis_linear = self.get_parameter('axis_linear').value
        self.axis_angular = self.get_parameter('axis_angular').value

        cmd_vel_topic = 'cmd_vel_safe'
        qos = QoSProfile(depth=10)
        self.publisher_ = self.create_publisher(Twist, cmd_vel_topic, qos)
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            qos)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.target_linear_velocity = 0.0
        self.target_angular_velocity = 0.0
        self.deadman_pressed = False

        self.get_logger().info('Teleop Joycon Node Started!')
        self.get_logger().info(f'Publishing to: {cmd_vel_topic}')
        self.get_logger().info('Controls:')
        self.get_logger().info('  Left Joystick (LS) : Move (Linear / Angular)')
        self.get_logger().info('  RT Trigger         : Deadman Switch (Must be held to move)')
        self.get_logger().info('---------------------------')

    def joy_callback(self, msg):
        deadman_pressed = False

        # Check RT as an Axis (Axis 5 is standard RT in ROS joy)
        # Typically 1.0 is unpressed, -1.0 is fully pressed.
        # We check if it's less than 0.0 to ensure it is significantly pressed
        # and ignore the 0.0 default initialization state if present.
        if len(msg.axes) > 5:
            if msg.axes[5] < 0.0:
                deadman_pressed = True

        # Check RT as a Button (Button 7 is often RT, Button 5 is RB)
        # We accept either 7 or 5 just to be robust across different controller modes
        if len(msg.buttons) > 7 and msg.buttons[7] == 1:
            deadman_pressed = True
        if len(msg.buttons) > 5 and msg.buttons[5] == 1:
            deadman_pressed = True

        self.deadman_pressed = deadman_pressed

        if self.deadman_pressed:
            if len(msg.axes) > self.axis_linear:
                self.target_linear_velocity = msg.axes[self.axis_linear] * self.max_lin_vel
            if len(msg.axes) > self.axis_angular:
                self.target_angular_velocity = msg.axes[self.axis_angular] * self.max_ang_vel
        else:
            self.target_linear_velocity = 0.0
            self.target_angular_velocity = 0.0

    def timer_callback(self):
        twist = Twist()
        twist.linear.x = float(self.target_linear_velocity)
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(self.target_angular_velocity)

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopJoycon()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(e)
    finally:
        # Publish stop message before shutting down
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        if rclpy.ok():
            node.publisher_.publish(twist)
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
