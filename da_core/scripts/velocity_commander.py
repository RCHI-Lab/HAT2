#!/usr/bin/env python3

from typing import Sequence

import rospy
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Float64MultiArray


class RealVelocityCommander:
    def __init__(self) -> None:
        import stretch_body.robot
        import stretch_body.stepper
        import stretch_body.end_of_arm
        self.r =stretch_body.robot.Robot()
        self.r.startup()
        assert(self.r.is_calibrated()) # the robot must be homed

    def pub_vel(self, q_dot: Sequence[float]):
        # arm_msg = Float64MultiArray(data=q_dot[2:])
        # self.arm_vel_pub.publish(arm_msg)
        # base_msg = Twist(linear=Vector3(q_dot[0], 0, 0), angular=Vector3(0, 0, q_dot[1]))
        # self.base_vel_pub.publish(base_msg)
        self.r.base.set_velocity(q_dot[0], q_dot[1])
        self.r.lift.set_velocity(q_dot[2])
        self.r.arm.set_velocity(sum(q_dot[3:7]))
        self.r.end_of_arm.get_joint('wrist_yaw').set_velocity(q_dot[7])
        self.r.end_of_arm.get_joint('stretch_gripper').set_velocity(q_dot[8])
        self.r.push_command()

    def stop(self):
        self.r.base.set_velocity(0, 0)
        self.r.lift.set_velocity(0)
        self.r.arm.set_velocity(0)
        self.r.end_of_arm.get_joint('wrist_yaw').set_velocity(0)
        self.r.end_of_arm.get_joint('stretch_gripper').set_velocity(0)
        self.r.push_command()

class SimVelocityCommander:
    def __init__(self) -> None:
        self.arm_vel_pub = rospy.Publisher(
            "/stretch_arm_controller/command", Float64MultiArray, queue_size=1
        )
        self.base_vel_pub = rospy.Publisher(
            "/stretch_diff_drive_controller/cmd_vel", Twist, queue_size=1
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
