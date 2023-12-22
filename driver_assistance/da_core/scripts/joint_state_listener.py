#!/usr/bin/env python3
from __future__ import annotations

import rospy
from sensor_msgs.msg import JointState


class JointStateListener:
    def __init__(self) -> None:
        rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)
        self.msg: JointState | None = None
        rospy.wait_for_message("/joint_states", JointState)

    def joint_state_cb(self, joint_states: JointState) -> None:
        self.msg = joint_states

    # publish rate is 50Hz in gazebo
    def get_state(self) -> tuple[rospy.Duration, tuple]:
        """get latest joint state"""
        if self.msg is None:
            raise ValueError("No joint state received yet")
        duration: rospy.Duration = rospy.get_rostime() - self.msg.header.stamp
        # name: - joint_arm_l0 - joint_arm_l1 - joint_arm_l2 - joint_arm_l3 - joint_gripper_finger_left
        #   - joint_gripper_finger_right - joint_head_pan - joint_head_tilt - joint_left_wheel
        #   - joint_lift - joint_right_wheel - joint_wrist_yaw
        q_modified = (
            0.0,
            0.0,
            self.msg.position[self.msg.name.index("joint_lift")],
            self.msg.position[self.msg.name.index("joint_arm_l3")],
            self.msg.position[self.msg.name.index("joint_arm_l2")],
            self.msg.position[self.msg.name.index("joint_arm_l1")],
            self.msg.position[self.msg.name.index("joint_arm_l0")],
            self.msg.position[self.msg.name.index("joint_wrist_yaw")],
        )
        return duration, q_modified


if __name__ == "__main__":
    rospy.init_node("joint_state_listener")
    listener = JointStateListener()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        try:
            duration, q = listener.get_state()
            rospy.loginfo_throttle(1, f"{q}, {duration.to_sec()}s ago")
        except ValueError as e:
            rospy.loginfo_throttle(1, e)
        rate.sleep()
