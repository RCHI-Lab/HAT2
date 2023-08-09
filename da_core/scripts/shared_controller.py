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

# isort: split
from da_core.msg import GoalBelief, GoalBeliefArray
from std_msgs.msg import Int16


class SharedController(ControllerBase):
    def __init__(
        self,
        da_enable_topic,
        fixed_joints=(),
        constraint: Literal["hard", "soft", "none"] = "none",
        clamp=False,
        max_speed=0.5,
        soft_max_vel=0.05,
        use_cofidence=False,
    ) -> None:
        super().__init__(uh_topic="/teleop_velocity_command")
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.fixed_joints = set(fixed_joints)
        self.constraint = constraint
        self.clamp = clamp
        self.max_speed = max_speed
        self.soft_max_vel = soft_max_vel
        self.use_confidence = use_cofidence
        self.goal_beliefs: dict[int, float] = {}
        self.sorted_goals: list[tuple[int, float]] = []
        self.goal_sub = rospy.Subscriber("/goal_beliefs", GoalBeliefArray, self.goal_beliefs_cb)
        self.enabled = True
        if rospy.get_param("interface") == "hat":
            self.enable_sub = rospy.Subscriber(da_enable_topic, Int16, self.da_enable_cb)
            self.enabled = False

    def da_enable_cb(self, msg: Int16):
        assert msg.data in (0, 1)
        self.enabled = bool(msg.data)

    def goal_beliefs_cb(self, msg: GoalBeliefArray) -> None:
        goal: GoalBelief
        for goal in msg.goals:
            self.goal_beliefs[goal.id] = goal.belief
        self.sorted_goals = sorted(self.goal_beliefs.items(), key=lambda x: x[1], reverse=True)

    @property
    def confidence(self):
        if len(self.sorted_goals) == 0:
            return None
        if len(self.sorted_goals) < 2:
            return self.sorted_goals[0][1]
        return self.sorted_goals[0][1] - self.sorted_goals[1][1]

    @property
    def current_goal(self):
        return None if len(self.sorted_goals) == 0 else self.sorted_goals[0][0]

    def auto_step(self):
        goal_id = self.current_goal
        if goal_id is None:
            return np.zeros(8)
        dur, q = self._jnt_state_listener.get_state()
        J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=self.fixed_joints, only_trans=True)
        err = self.get_err(target_frame=f"goal{goal_id}")
        if err is None:
            return np.zeros(8)
        q_dot = J_pinv @ err[:3]
        return self.clamp_qd(q_dot) if self.clamp else q_dot

    def combine(self, ur):
        if (not self.use_confidence) and np.linalg.norm(self.uh) <= 0:
            return np.zeros(ur.shape)

        constraint_joints = [i for i in range(8) if self.uh[i] != 0]
        if self.constraint == "soft":
            ur[constraint_joints] = np.clip(
                ur[constraint_joints], -self.soft_max_vel, self.soft_max_vel
            )
        elif self.constraint == "hard":
            ur[constraint_joints] = np.zeros(len(constraint_joints))

        if not self.use_confidence:
            return ur + self.uh

        if self.confidence is None:
            return self.uh

        rospy.loginfo_throttle(
            0.5, f"goal: {self.current_goal}, confidence: {self.confidence:.3f}\r"
        )
        return self.confidence * ur + self.uh

    @override
    def step(self):
        if self.enabled:
            self._vel_cmder.pub_vel(self.uh)
        ur = self.auto_step()
        q_dot = self.combine(ur)
        self._vel_cmder.pub_vel(q_dot)


if __name__ == "__main__":
    rospy.init_node("shared_controller")
    ctrler = SharedController(
        da_enable_topic="/da", fixed_joints=(1, 7), constraint="soft", use_cofidence=True
    )
    r = rospy.Rate(30)

    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ctrler.step()
            r.sleep()
