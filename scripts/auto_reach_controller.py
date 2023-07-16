#!/usr/bin/env python3

from contextlib import suppress

import numpy as np
import rospy
from controller_base import ControllerBase
from joint_state_listener import JointStateListener
from stretch_ik_solver import IKSolver
from typing_extensions import override


class AutoReachController(ControllerBase):
    def __init__(self, fixed_joints=(), clamp=False, max_speed=0.5) -> None:
        super().__init__(uh_topic=None)
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.fixed_joints = fixed_joints
        self.clamp = clamp
        self.max_speed = max_speed

    @override
    def step(self):
        dur, q = self._jnt_state_listener.get_state()
        J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=self.fixed_joints, only_trans=True)
        err = np.array(self.get_err(target_frame="goal0"))
        q_dot = J_pinv @ err[:3]
        if self.clamp:
            if (length := np.linalg.norm(q_dot)) > self.max_speed:
                rospy.loginfo_throttle(1, length)
                q_dot = q_dot / length * self.max_speed
        self._vel_cmder.pub_vel(q_dot)
        # rospy.loginfo_throttle(1, q_dot)


if __name__ == "__main__":
    rospy.init_node("auto_reach_controller")
    ctrler = AutoReachController(fixed_joints=(0, 7), clamp=True)
    r = rospy.Rate(30)
    rospy.sleep(0.5)

    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ctrler.step()
            r.sleep()
