#!/usr/bin/env python3

from __future__ import annotations

import rospy
from std_msgs.msg import Float64, Float64MultiArray
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
import hello_helpers.hello_misc as hm


class StretchController(hm.HelloNode):
    def __init__(self) -> None:
        hm.HelloNode.__init__(self)
        self.gripper_range = (-0.35, 0.165)  # in radians

    def joint_cb(self, msg: Float64MultiArray) -> None:
        q_dot = msg.data
        # base
        base_msg = Twist(linear=Vector3(q_dot[0], 0, 0), angular=Vector3(0, 0, q_dot[1]))
        self.base_vel_pub.publish(base_msg)
        # arm
        self.move_at_speed(
            {
                "joint_lift": q_dot[2],
                "joint_arm": sum(q_dot[3:7]),
                "joint_wrist_yaw": q_dot[7],
            },
            # return_before_done=True,
        )

    def gripper_cb(self, msg: Float64):
        # divide or multiply by some scaling factor that you can find through testing
        gripper_delta = msg.data * 0.005
        self.gripper += gripper_delta
        self.gripper = hm.bound_ros_command(
            self.gripper_range, self.gripper, fail_out_of_range_goal=False
        )
        # must return before done to constantly update gripper goal position
        self.move_to_pose({"joint_gripper_finger_left": self.gripper}, return_before_done=True)

    def main(self):
        hm.HelloNode.main(
            self, "stretch_controller", "stretch_controller", wait_for_first_pointcloud=False
        )
        rospy.Subscriber("/stretch_controller/joint_cmd", Float64MultiArray, self.joint_cb)
        rospy.Subscriber("/stretch_controller/gripper_cmd", Float64, self.gripper_cb)
        self.base_vel_pub = rospy.Publisher("/stretch/cmd_vel", Twist, queue_size=1)
        js_msg = rospy.wait_for_message("/joint_states", JointState)
        self.gripper = js_msg.position[js_msg.name.index("joint_gripper_finger_left")]
        rospy.spin()


if __name__ == "__main__":
    ctrler = StretchController()
    ctrler.main()
