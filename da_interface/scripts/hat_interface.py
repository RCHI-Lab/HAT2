#!/usr/bin/env python3

from __future__ import annotations

import rospy
from std_msgs.msg import Float64, Float64MultiArray


class HatInterface:
    def __init__(self) -> None:
        rospy.Subscriber("/velocities", Float64MultiArray, self.joint_state_cb)
        self.publisher = rospy.Publisher(
            "/teleop_velocity_command", Float64MultiArray, queue_size=1
        )
        self.gripper_publisher = rospy.Publisher(
            "/stretch_controller/gripper_cmd", Float64, queue_size=1
        )

    def joint_state_cb(self, msg: Float64MultiArray) -> None:
        vel = msg.data
        assert len(vel) == 6
        self.publisher.publish(Float64MultiArray(data=[*vel[:3], *([vel[3] / 4] * 4), vel[4]]))
        self.gripper_publisher.publish(Float64(data=vel[5]))


if __name__ == "__main__":
    rospy.init_node("hat_interface")
    interface = HatInterface()
    rospy.spin()
