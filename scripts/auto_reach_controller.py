#!/usr/bin/env python3

import numpy as np
import rospy
from joint_state_listener import JointStateListener
from shared_autonomy_controller import ControllerBase
from stretch_ik_solver import IKSolver
from typing_extensions import override


class AutoReachController(ControllerBase):
    def __init__(self, fixed_joints=()) -> None:
        super().__init__(uh_topic=None)
        self._ik_solver = IKSolver()
        self._jnt_state_listener = JointStateListener()
        self.fixed_joints = fixed_joints

    @override
    def step(self):
        dur, q = self._jnt_state_listener.get_state()
        J_pinv = self._ik_solver.solve_J_pinv(q, fixed_joints=self.fixed_joints)
        err = np.array(self.get_err())
        q_dot = J_pinv @ err
        self._vel_cmder.pub_vel(q_dot)
        rospy.loginfo_throttle(1, q_dot)


if __name__ == "__main__":
    rospy.init_node("auto_reach_controller")
    ctrler = AutoReachController(fixed_joints=())
    r = rospy.Rate(30)
    rospy.sleep(0.5)

    while not rospy.is_shutdown():
        ctrler.step()
        r.sleep()
