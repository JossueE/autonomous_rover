#!/usr/bin/env python3

""" 
Node to move, sending control velocities (V,W), a differential drive mobile robot
saturating maximum velocities.

WARNING: This node DOES NOT publish the cmd_vel with steps. Ramps of speed up/down must be implemented
in another node.

To finish this node, please press 'ctrl + c'."""

__author__ = "C. Mauricio Arteaga-Escamilla"

import os, select, sys, rclpy
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile

if os.name == 'nt':
    import msvcrt
else:
    import termios
    import tty

MAX_LIN_VEL = 0.7
MAX_ANG_VEL = 0.8

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Diff-drive Mobile Robot!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity (Max vel: 0.8)
a/d : increase/decrease angular velocity (Max vel: 0.5)
p : Max linear vel

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key(settings):
    if os.name == 'nt':
        return msvcrt.getch().decode('utf-8')
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def print_vels(target_linear_velocity, target_angular_velocity):
    print('currently:\tV: {0}\t W: {1} '.format(
        target_linear_velocity,
        target_angular_velocity))


def constrain(input_vel, low_bound, high_bound):
    if input_vel < low_bound:
        input_vel = low_bound
    elif input_vel > high_bound:
        input_vel = high_bound
    else:
        input_vel = input_vel

    return input_vel


def check_linear_limit_velocity(velocity):
    return constrain(velocity, -MAX_LIN_VEL, MAX_LIN_VEL)

def check_angular_limit_velocity(velocity):
    return constrain(velocity, -MAX_ANG_VEL, MAX_ANG_VEL)


def main():
    settings = None
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()

    qos = QoSProfile(depth=10)
    node = rclpy.create_node('teleop_keyboard')
    cmd_vel_topic = 'cmd_vel'
    pub = node.create_publisher(Twist, cmd_vel_topic, qos)

    status = 0
    target_linear_velocity = 0.0
    target_angular_velocity = 0.0
    # control_linear_velocity = 0.0
    # control_angular_velocity = 0.0

    node.get_logger().warn('By default, publishing on topic: '+ cmd_vel_topic)
    node.get_logger().info('To use a namespace, to remap topics, services and node name, please use:')
    node.get_logger().warn('--ros-args -r __ns:=/new_ns')
    node.get_logger().info('To remmap the cmd_vel topic, use:')
    node.get_logger().warn('--ros-args -r cmd_vel:=/cmd_vel_safe')

    try:
        print(msg)
        while(1):
            key = get_key(settings)
            if key == 'w' or key == 'W':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity + LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'x' or key == 'X':
                target_linear_velocity =\
                    check_linear_limit_velocity(target_linear_velocity - LIN_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'p' or key == 'P':
                target_linear_velocity = MAX_LIN_VEL
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'a' or key == 'A':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity + ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == 'd' or key == 'D':
                target_angular_velocity =\
                    check_angular_limit_velocity(target_angular_velocity - ANG_VEL_STEP_SIZE)
                status = status + 1
                print_vels(target_linear_velocity, target_angular_velocity)
            elif key == ' ' or key == 's'  or key == 'S':
                target_linear_velocity = 0.0
                # control_linear_velocity = 0.0
                target_angular_velocity = 0.0
                # control_angular_velocity = 0.0
                print_vels(target_linear_velocity, target_angular_velocity)
            else:
                if (key == '\x03'):
                    break

            if status == 20:
                print(msg)
                status = 0

            twist = Twist()


            twist.linear.x = target_linear_velocity
            twist.linear.y = 0.0
            twist.linear.z = 0.0

            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_velocity

            pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0

        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)

        if os.name != 'nt':
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


if __name__ == '__main__':
    main()
