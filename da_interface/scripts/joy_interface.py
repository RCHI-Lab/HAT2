#!/usr/bin/env python3

from __future__ import annotations

import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Float64MultiArray


class JoyconInterface:
    def __init__(self) -> None:
        rospy.Subscriber("/joy", Joy, self.interface_cb)
        self.publisher = rospy.Publisher("teleop_velocity_command", Float64MultiArray, queue_size=1)
        self.gripper_publisher = rospy.Publisher(
            "/stretch_controller/gripper_cmd", Float64, queue_size=1
        )
        self.axis_assign = [1, 0, 4, 3, 6, 7]  # base trans, base turn, lift, arm, wrist, gripper
        self.axis_factors = [0.25, 0.75, 0.3, -0.05, 0.5, 1]

    def interface_cb(self, msg: Joy) -> None:
        vel = [0] * 6
        print(msg.axes)
        for idx in range(6):
            vel[idx] = msg.axes[self.axis_assign[idx]] * self.axis_factors[idx]
        self.publisher.publish(Float64MultiArray(data=[*vel[:3], *[vel[3]] * 4, vel[4]]))
        self.gripper_publisher.publish(Float64(data=vel[5]))


if __name__ == "__main__":
    rospy.init_node("joy_interface")
    interface = JoyconInterface()
    rospy.spin()
