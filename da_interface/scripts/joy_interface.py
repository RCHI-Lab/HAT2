#!/usr/bin/env python3

from __future__ import annotations

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class JoyconInterface:
    def __init__(self) -> None:
        rospy.Subscriber("/joy", Joy, self.interface_cb)
        self.publisher = rospy.Publisher("teleop_velocity_command", Float64MultiArray, queue_size=1)
        self.axis_assign = [3, 2, 1, 0, 4]  # base trans, base turn, lift, arm, wrist
        self.axis_factors = [0.25, 0.75, 1, -0.25, 3]

    def interface_cb(self, msg: Joy) -> None:
        vel = [0] * 5
        print(msg.axes)
        for idx in range(5):
            vel[idx] = msg.axes[self.axis_assign[idx]] * self.axis_factors[idx]
        self.publisher.publish(Float64MultiArray(data=[*vel[:3], *[vel[3]] * 4, vel[4], 0]))


if __name__ == "__main__":
    rospy.init_node("joy_interface")
    interface = JoyconInterface()
    rospy.spin()
