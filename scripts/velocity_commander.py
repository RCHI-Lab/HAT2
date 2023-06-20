#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray


class VelocityCommander:
    def __init__(self) -> None:
        self.arm_vel_pub = rospy.Publisher(
            "/stretch_arm_controller/command", Float64MultiArray, queue_size=1
        )
        self.base_vel_pub = rospy.Publisher(
            "/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=1
        )

    def pub_vel(self, qdot):
        arm_msg = Float64MultiArray(data=qdot[2:])
        self.arm_vel_pub.publish(arm_msg)
        base_msg = Twist(linear=Vector3(qdot[0], 0, 0), angular=Vector3(0, 0, qdot[1]))
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
    vel_cmder = VelocityCommander()

    r = rospy.Rate(10)
    i = 1
    j = 0
    rospy.on_shutdown(vel_cmder.stop)
    while not rospy.is_shutdown():
        if j == 30:
            if i == -1:
                i = 1
            else:
                i = -1
            j = 0
        j = j + 1
        vel_cmder.pub_vel([i * x for x in [0.5, 0.25, 0.2, 0.04, 0.04, 0.04, 0.04, 1]])
        r.sleep()
