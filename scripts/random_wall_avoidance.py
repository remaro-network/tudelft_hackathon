#!/usr/bin/env python
import sys
import rclpy
import random
import threading

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from mavros_wrapper.ardusub_wrapper import *

def laser_scan_cb(msg, ardusub):
    min_distance = 1.5
    yaw_speed = 0.3
    forward_speed = 0.15
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

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    service_timer = ardusub.create_rate(2)
    while ardusub.status.mode != "ALT_HOLD":
        ardusub.set_mode("ALT_HOLD")
        service_timer.sleep()

    print("ALT HOLD mode selected")

    while ardusub.status.armed == False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    ardusub.toogle_rc_override(True)
    ardusub.set_rc_override_channels(forward=0.35)

    laser_subscriber = ardusub.create_subscription(
        LaserScan, '/scan', (lambda msg: laser_scan_cb(msg, ardusub)), 10)

    rate = ardusub.create_rate(2)
    try:
        while rclpy.ok():
            rate.sleep()
    except KeyboardInterrupt:
        pass

    ardusub.destroy_node()
    rclpy.shutdown()
