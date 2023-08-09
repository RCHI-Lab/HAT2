#!/usr/bin/env python3

from __future__ import annotations

from contextlib import suppress
from typing import Literal, Sequence

import numpy as np
import rospy
from controller_base import ControllerBase
from joint_state_listener import JointStateListener
from stretch_ik_solver import IKSolver

# isort: split
from da_core.msg import GoalBelief, GoalBeliefArray
from std_msgs.msg import Int16
from std_srvs.srv import Empty


class SharedController(ControllerBase):
    def __init__(
        self,
        uh_topic: str,
        da_enable_topic: str,
        ik_fixed_joints: Sequence[int],
        fixed_joints: list[int],
        constraint: Literal["hard", "soft", "none"] = "none",
        clamp=False,
        max_speed=0.5,
        soft_max_vel=0.05,
        use_confidence=False,
    ) -> None:
        super().__init__(uh_topic)
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.ik_fixed_joints = set(ik_fixed_joints)
        self.fixed_joints = fixed_joints
        self.constraint = constraint
        self.clamp = clamp
        self.max_speed = max_speed
        self.soft_max_vel = soft_max_vel
        self.use_confidence = use_confidence
        self.goal_beliefs: dict[int, float] = {}
        self.sorted_goals: list[tuple[int, float]] = []
        self.goal_sub = rospy.Subscriber("/goal_beliefs", GoalBeliefArray, self.goal_beliefs_cb)
        self.enabled = True
        if rospy.get_param("interface") == "hat":
            self.enable_sub = rospy.Subscriber(da_enable_topic, Int16, self.da_enable_cb)
            self.enabled = False
            rospy.wait_for_service("stretch_controller/open_gripper")

    def da_enable_cb(self, msg: Int16) -> None:
        assert msg.data in (0, 1)
        self.enabled = bool(msg.data)
        if self.enabled:
            try:
                open_gripper = rospy.ServiceProxy("stretch_controller/open_gripper", Empty)
                open_gripper()
            except rospy.ServiceException as e:
                print(f"Service call failed: {e}")

    def goal_beliefs_cb(self, msg: GoalBeliefArray) -> None:
        goal: GoalBelief
        for goal in msg.goals:
            self.goal_beliefs[goal.id] = goal.belief
        self.sorted_goals = sorted(self.goal_beliefs.items(), key=lambda x: x[1], reverse=True)

    @property
    def confidence(self) -> float | None:
        if len(self.sorted_goals) == 0:
            return None
        if len(self.sorted_goals) == 1:
            return self.sorted_goals[0][1]
        return self.sorted_goals[0][1] - self.sorted_goals[1][1]

    @property
    def current_goal(self) -> int | None:
        return None if len(self.sorted_goals) == 0 else self.sorted_goals[0][0]

    def auto_step(self) -> np.ndarray:
        goal_id = self.current_goal
        if goal_id is None:
            return np.zeros(8)
        dur, q = self._jnt_state_listener.get_state()
        J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=self.ik_fixed_joints, only_trans=True)
        err = self.get_err(target_frame=f"goal{goal_id}")
        if err is None:
            return np.zeros(8)
        q_dot = J_pinv @ err[:3]
        return self.clamp_qd(q_dot) if self.clamp else q_dot

    def combine(self, ur: np.ndarray) -> np.ndarray:
        # stop move if human input is zero and we are not using confidence
        if (not self.use_confidence) and np.linalg.norm(self.uh) <= 0:
            return np.zeros(ur.shape)

        # constraints
        constraint_joints = [i for i in range(8) if self.uh[i] != 0]
        if self.constraint == "soft":
            ur[constraint_joints] = np.clip(
                ur[constraint_joints], -self.soft_max_vel, self.soft_max_vel
            )
        elif self.constraint == "hard":
            ur[constraint_joints] = np.zeros(len(constraint_joints))

        ur[self.fixed_joints] = np.zeros(len(self.fixed_joints))

        # combine
        if not self.use_confidence:
            return ur + self.uh
        if self.confidence is None:
            return self.uh
        rospy.loginfo_throttle(
            0.5, f"goal: {self.current_goal}, confidence: {self.confidence:.3f}\r"
        )
        return self.confidence * ur + self.uh

    def step(self) -> None:
        if not self.enabled:
            # return human input if da is disabled
            self._vel_cmder.pub_vel(self.uh)
            return
        ur = self.auto_step()
        ur /= 1
        ur[[3, 4, 5, 6]] /= 5
        q_dot = self.combine(ur)
        self._vel_cmder.pub_vel(q_dot)


if __name__ == "__main__":
    rospy.init_node("shared_controller")
    ctrler = SharedController(
        uh_topic="/teleop_velocity_command",
        da_enable_topic="/da",
        ik_fixed_joints=(0, 7),
        fixed_joints=[0, 7],
        constraint="soft",
        use_confidence=True,
    )
    r = rospy.Rate(30)

    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ctrler.step()
            r.sleep()
