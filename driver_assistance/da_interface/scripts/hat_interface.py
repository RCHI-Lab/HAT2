#!/usr/bin/env python3

from __future__ import annotations

import rospy
from std_msgs.msg import Float64, Float64MultiArray
stop = False

class HatInterface:
    def __init__(self) -> None:
        rospy.Subscriber("velocities", Float64MultiArray, self.interface_cb)
        self.joints_pub = rospy.Publisher(
            "teleop_velocity_command", Float64MultiArray, queue_size=1
        )
        self.gripper_pub = rospy.Publisher("stretch_controller/gripper_cmd", Float64, queue_size=1)
        self.vel = [0,0,0,0,0,0]
        self.last_time = rospy.get_time()

    def interface_cb(self, msg: Float64MultiArray) -> None:
        self.vel = msg.data
        assert len(self.vel) == 6
        interface.last_time = rospy.get_time()
        


if __name__ == "__main__":
    rospy.init_node("hat_interface")
    interface = HatInterface()

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():
        curr_time = rospy.get_time()
        interface.joints_pub.publish(Float64MultiArray(data=[*interface.vel[:3], *([interface.vel[3] / 4] * 4), interface.vel[4]]))
        interface.gripper_pub.publish(Float64(data=interface.vel[5]))
        print(interface.vel)
        if stop == True: 
            c = input('enter:')
            if c == 'c':
                print()
                print()
                print('Continuing')
                stop = False
                interface.last_time = rospy.get_time()
                curr_time = rospy.get_time()
        if curr_time - interface.last_time > 1.0:
            print("Robot Stopped")
            interface.vel = [0,0,0,0,0,0]
            stop = True
        rate.sleep()
    
    rospy.spin()
