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
        if uh_topic is None:
            del self.uh_cb
        else:
            rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
            self.uh = np.zeros(8)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._vel_cmder = VelocityCommander()
        self.verbose = verbose

    def uh_cb(self, data: Float64MultiArray) -> None:
        """human input callback function"""
        self.uh = np.array(data.data)
        if self.verbose:
            rospy.loginfo(f"received: {self.uh}")

    @abc.abstractmethod
    def step(self):
        pass

    def get_err(self, target_frame="ee_goal"):
        try:
            transfrom: TransformStamped = self._tf_buffer.lookup_transform(
                target_frame, "link_grasp_center", rospy.Time()
            )
            translation = transfrom.transform.translation
            return np.array([translation.x, translation.y, translation.z])
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(e)

    def publish_vel(self, q_dot: Sequence[float]):
        self._vel_cmder.pub_vel(q_dot)
