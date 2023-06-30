#!/usr/bin/env python3

from __future__ import annotations

from typing import Literal

import numpy as np
import rospy
from controller_base import ControllerBase
from joint_state_listener import JointStateListener
from stretch_ik_solver import IKSolver
from typing_extensions import override


class AutoReachController(ControllerBase):
    def __init__(
        self, fixed_joints=(), constraint: Literal["soft", "none"] = "none", max_vel=0.05
    ) -> None:
        super().__init__(uh_topic="/teleop_velocity_command")
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.fixed_joints = set(fixed_joints)
        self.constraint = constraint
        self.max_vel = max_vel

    @override
    def step(self):
        ur = self.auto_step()
        q_dot = self.combine(ur)
        self._vel_cmder.pub_vel(q_dot)
        rospy.loginfo_throttle(1, q_dot)

    def auto_step(self):
        dur, q = self._jnt_state_listener.get_state()
        if self.constraint == "hard":
            fixed_joints = self.fixed_joints | set([i for i in range(8) if self.uh[i] != 0])
            J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=fixed_joints, only_trans=True)
        else:
            J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=self.fixed_joints, only_trans=True)
        err = np.array(self.get_err(target_frame="ee_goal2"))
        ur = J_pinv @ err[:3]
        return ur

    def combine(self, ur):
        if np.linalg.norm(self.uh) > 0:
            if self.constraint == "soft":
                soft_joints = [i for i in range(8) if self.uh[i] != 0]
                if soft_joints:
                    ur[soft_joints] = np.clip(ur[soft_joints], -self.max_vel, self.max_vel)
            q_dot = ur + self.uh
        else:
            q_dot = np.zeros(ur.shape)
        return q_dot


if __name__ == "__main__":
    rospy.init_node("shared_controller")
    ctrler = AutoReachController(fixed_joints=(), constraint="soft")
    r = rospy.Rate(30)
    rospy.sleep(0.5)

    try:
        while not rospy.is_shutdown():
            ctrler.step()
            r.sleep()
    except rospy.ROSInterruptException:
        pass
