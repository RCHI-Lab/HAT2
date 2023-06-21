#!/usr/bin/env python3

from __future__ import annotations

import rospy
from shared_autonomy_controller import ControllerBase
from typing_extensions import override


class TeleopController(ControllerBase):
    def __init__(self, uh_topic="/teleop_velocity_command") -> None:
        super().__init__(uh_topic)

    @override
    def step(self):
        self._vel_cmder.pub_vel(self.uh)


if __name__ == "__main__":
    rospy.init_node("teleop_controller")
    ctrler = TeleopController()
    r = rospy.Rate(30)

    while not rospy.is_shutdown():
        ctrler.step()
        r.sleep()
