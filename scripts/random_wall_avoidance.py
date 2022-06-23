#!/usr/bin/env python
import sys
import rclpy
import random
import threading

from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from mavros_wrapper.ardusub_wrapper import *

def laser_scan_cb(msg, ardusub):
    min_distance = 1.25
    yaw_speed = 0.3
    forward_speed = 0.12

    # TODO: Do something with the sonar msg in order to make the robot not
    # crash into the walls
    #
    # CHEAT:
    # LaserScan msg definition can be found here: https://docs.ros2.org/latest/api/sensor_msgs/msg/LaserScan.html
    # Before checking the solution provided, check this example:
    # https://github.com/gazebosim/docs/blob/master/citadel/sensors.md#avoid-the-wall
    for scan in msg.ranges:
        pass


if __name__ == '__main__':
    print("Starting wall avoidance. Let's swim!")

    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("ardusub_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    # TODO: Set flight mode to MANUAL, STABILIZE or DEPTH HOLD

    print("Manual mode selected")

    # TODO: Arm motors

    print("Thrusters armed")

    print("Initializing mission")

    # TODO: start publishing on /mavros/rc/override

    # TODO: start moving forward

    # Sonar subscriber
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
