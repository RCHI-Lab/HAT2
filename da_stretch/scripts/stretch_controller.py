#!/usr/bin/env python3

import rospy
import stretch_body.end_of_arm
import stretch_body.robot
import stretch_body.stepper
from std_msgs.msg import Float64, Float64MultiArray


class StretchController:
    def __init__(self) -> None:
        self.r = stretch_body.robot.Robot()
        self.r.startup()
        assert self.r.is_calibrated()  # the robot must be homed
        rospy.Subscriber("/stretch_controller/joint_cmd", Float64MultiArray, self.joint_cb)
        rospy.Subscriber("/stretch_controller/gripper_cmd", Float64, self.gripper_cb)

    def joint_cb(self, msg: Float64MultiArray) -> None:
        q_dot = msg.data
        self.r.base.set_velocity(q_dot[0], q_dot[1])
        self.r.lift.set_velocity(q_dot[2])
        self.r.arm.set_velocity(sum(q_dot[3:7]))
        self.r.end_of_arm.get_joint("wrist_yaw").set_velocity(q_dot[7])
        self.r.push_command()

    def gripper_cb(self, msg: Float64):
        self.r.end_of_arm.get_joint("stretch_gripper").set_velocity(msg.data)
        self.r.push_command()


if __name__ == "__main__":
    rospy.init_node("stretch_controller")
    ctrler = StretchController()
    rospy.spin()
