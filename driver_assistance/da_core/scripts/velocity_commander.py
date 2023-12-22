#!/usr/bin/env python3

from typing import Sequence

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray


class RealVelocityCommander:
    def __init__(self) -> None:
        self.pub = rospy.Publisher("stretch_controller/joint_cmd", Float64MultiArray, queue_size=1)

    def pub_vel(self, q_dot: Sequence[float]):
        assert len(q_dot) == 8
        rospy.logdebug_throttle(1, f"velocity commander: {q_dot}")
        self.pub.publish(data=q_dot)

    def stop(self):
        stop_msg = Float64MultiArray(data=[0] * 8)
        self.pub.publish(stop_msg)


class SimVelocityCommander:
    def __init__(self) -> None:
        self.arm_vel_pub = rospy.Publisher(
            "stretch_arm_controller/command", Float64MultiArray, queue_size=1
        )
        self.base_vel_pub = rospy.Publisher(
            "stretch_diff_drive_controller/cmd_vel", Twist, queue_size=1
        )

    def pub_vel(self, q_dot: Sequence[float]):
        arm_msg = Float64MultiArray(data=q_dot[2:])
        self.arm_vel_pub.publish(arm_msg)
        base_msg = Twist(linear=Vector3(q_dot[0], 0, 0), angular=Vector3(0, 0, q_dot[1]))
        self.base_vel_pub.publish(base_msg)

    def stop(self):
        arm_stop_msg = Float64MultiArray(data=[0] * 6)
        base_stop_msg = Twist(linear=Vector3(0, 0, 0), angular=Vector3(0, 0, 0))
        r = rospy.Rate(10)
        for _ in range(3):
            self.arm_vel_pub.publish(arm_stop_msg)
            self.base_vel_pub.publish(base_stop_msg)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("velocity_commander")
    vel_cmder = SimVelocityCommander()

    r = rospy.Rate(10)
    i = 1
    j = 0
    rospy.on_shutdown(vel_cmder.stop)
    while not rospy.is_shutdown():
        if j == 30:
            i = 1 if i == -1 else -1
            j = 0
        j = j + 1
        vel_cmder.pub_vel([i * x for x in [0.5, 0.25, 0.2, 0.04, 0.04, 0.04, 0.04, 1]])
        r.sleep()
