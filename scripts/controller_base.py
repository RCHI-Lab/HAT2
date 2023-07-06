from __future__ import annotations

import abc
from typing import Sequence

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from velocity_commander import VelocityCommander


class ControllerBase(abc.ABC):
    def __init__(self, uh_topic, verbose=False) -> None:
        if uh_topic is not None:
            rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
            self.uh = np.zeros(8)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._vel_cmder = VelocityCommander()
        self.verbose = verbose
        rospy.on_shutdown(self.stop)

    def uh_cb(self, data: Float64MultiArray) -> None:
        """human input callback function"""
        self.uh = np.array(data.data)
        if self.verbose:
            rospy.loginfo(f"received: {self.uh}")

    @abc.abstractmethod
    def step(self):
        pass

    # TODO refactor, function to get both target and ee position
    def get_err(self, target_frame="goal", base_frame="base_link") -> tuple | None:
        try:
            target_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, target_frame, rospy.Time(), timeout=rospy.Duration(secs=5)
            )
            target_pos = target_transform.transform.translation
            ee_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, "link_grasp_center", rospy.Time(), timeout=rospy.Duration(secs=5)
            )
            ee_pos = ee_transform.transform.translation
            return (
                target_pos.x - ee_pos.x,
                target_pos.y - ee_pos.y,
                target_pos.z - ee_pos.z,
                0,
                0,
                0,
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(e)
            return None

    def publish_vel(self, q_dot: Sequence[float]):
        self._vel_cmder.pub_vel(q_dot)

    def stop(self):
        self._vel_cmder.stop()
