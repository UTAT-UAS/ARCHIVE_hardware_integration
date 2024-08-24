#!/usr/bin/env python3

# barebones script for testing interface alone

import rospy
from hardware_integration.interface import HardwareInterface

if __name__ == "__main__":
    rospy.init_node("Interface_Test")
    rate = rospy.Rate(20)

    hw = HardwareInterface()
    hw.initialize_connection()

    hw.set_target_position_rel(hw.start_pose.pose.position, 0, 0, 4)
    hw.set_target_orientation(hw.start_pose.pose.orientation)

    while (not rospy.is_shutdown()):
        hw.cycle()

        print(hw.get_flight_info())
        rate.sleep()
