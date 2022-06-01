#!/usr/bin/env python
import sys
import rclpy
import random
import threading

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from mavros_wrapper.ardusub_wrapper import *
from mavros_wrapper.mavros_wrapper import *

def laser_scan_cb(msg, ardusub):
    min_distance = 1
    yaw_speed = 0.08
    forward_speed = 0.05
    allGreater = True
    for scan in msg.ranges:
        if scan < min_distance:
            allGreater = False
            _yaw_speed = (-1)**random.choice([1, 2])*yaw_speed
            ardusub.set_rc_override_channels(
                forward=-forward_speed/4,
                yaw=_yaw_speed)
            break
    if allGreater:
        ardusub.set_rc_override_channels(forward=forward_speed)


if __name__ == '__main__':
    print("Starting wall avoidance. Let's swim!")

    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("ardusub_node")
    ardusub.toogle_rc_override(True)

    laser_subscriber = ardusub.create_subscription(
        LaserScan, '/scan', (lambda msg: laser_scan_cb(msg, ardusub)), 10)

    rclpy.spin(ardusub)
    ardusub.destroy_node()
    rclpy.shutdown()
