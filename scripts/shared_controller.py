#!/usr/bin/env python3

from __future__ import annotations

from contextlib import suppress
from typing import Literal

import numpy as np
import rospy
from controller_base import ControllerBase
from joint_state_listener import JointStateListener
from stretch_ik_solver import IKSolver
from typing_extensions import override

from driver_assistance.msg import GoalBelief, GoalBeliefArray


class SharedController(ControllerBase):
    def __init__(
        self, fixed_joints=(), constraint: Literal["hard", "soft", "none"] = "none", max_vel=0.05
    ) -> None:
        super().__init__(uh_topic="/teleop_velocity_command")
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.fixed_joints = set(fixed_joints)
        self.constraint = constraint
        self.max_vel = max_vel
        self.goal_sub = rospy.Subscriber("/goal_beliefs", GoalBeliefArray, self.goal_beliefs_cb)
        self.goal_beliefs: dict[int, float] = {}
        self.sorted_goals: list[tuple[int, float]] = []

    def goal_beliefs_cb(self, msg: GoalBeliefArray) -> None:
        goal: GoalBelief
        for goal in msg.goals:
            self.goal_beliefs[goal.id] = goal.belief
        self.sorted_goals = sorted(self.goal_beliefs.items(), key=lambda x: x[1], reverse=True)
        
    @property
    def confidence(self):
        if len(self.sorted_goals) < 2:
            return 1.0
        return self.sorted_goals[0][1] - self.sorted_goals[1][1]
    
    @property
    def current_goal(self):
        if len(self.sorted_goals) == 0:
            raise ValueError("No goal belief received")
        return self.sorted_goals[0][0]

    def auto_step(self):
        dur, q = self._jnt_state_listener.get_state()
        if self.constraint == "hard":
            fixed_joints = self.fixed_joints | {i for i in range(8) if self.uh[i] != 0}
            J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=fixed_joints, only_trans=True)
        else:
            J_pinv = self._ik_solver.solve_J_pinv(
                q, fixed_joints=self.fixed_joints, only_trans=True
            )
        err = np.array(self.get_err(target_frame=f"goal{self.current_goal}"))
        return J_pinv @ err[:3]

    def combine(self, ur):
        if np.linalg.norm(self.uh) <= 0:
            return np.zeros(ur.shape)
        if self.constraint == "soft":
            # clip the joint velocity where uh is not 0
            if soft_joints := [i for i in range(8) if self.uh[i] != 0]:
                ur[soft_joints] = np.clip(ur[soft_joints], -self.max_vel, self.max_vel)
        return ur + self.confidence * self.uh

    @override
    def step(self):
        ur = self.auto_step()
        q_dot = self.combine(ur)
        self._vel_cmder.pub_vel(q_dot)
        rospy.loginfo_throttle(1, q_dot)


if __name__ == "__main__":
    rospy.init_node("shared_controller")
    ctrler = SharedController(fixed_joints=(), constraint="soft")
    r = rospy.Rate(30)
    rospy.sleep(0.5)

    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ctrler.step()
            r.sleep()
