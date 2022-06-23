#!/usr/bin/env python
import sys
import rclpy
from rclpy.node import Node
import threading

from mavros_wrapper.ardusub_wrapper import *

def mission(ardusub):

    service_timer = ardusub.create_rate(2)
    while ardusub.status.mode != "MANUAL":
        ardusub.set_mode("MANUAL")
        service_timer.sleep()

    print("Manual mode selected")

    while ardusub.status.armed == False:
        ardusub.arm_motors(True)
        service_timer.sleep()

    print("Thrusters armed")

    print("Initializing mission")

    timer = ardusub.create_rate(0.5) # Hz

    ardusub.toogle_rc_override(True)
    ardusub.set_rc_override_channels(forward=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(forward=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=-0.5)
    timer.sleep()
    ardusub.set_rc_override_channels(lateral=0)
    ardusub.toogle_rc_override(False)

    print("Mission completed")

if __name__ == '__main__':
    print("Starting Bluerov agent node")

    # Initialize ros node
    rclpy.init(args=sys.argv)

    ardusub = BlueROVArduSubWrapper("ardusub_node")

    thread = threading.Thread(target=rclpy.spin, args=(ardusub, ), daemon=True)
    thread.start()

    mission(ardusub)

    ardusub.destroy_node()
    rclpy.shutdown()
    thread.join()
