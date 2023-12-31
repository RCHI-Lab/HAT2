from __future__ import annotations

import abc
from ast import Not
from typing import Sequence

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from velocity_commander import RealVelocityCommander, SimVelocityCommander


class ControllerBase(abc.ABC):
    def __init__(self, uh_topic: str, verbose=False) -> None:
        self.verbose = verbose
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self._vel_cmder = (
            RealVelocityCommander()
            if rospy.get_param("/use_real_robot", False)
            else SimVelocityCommander()
        )
        if uh_topic is not None:
            rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
            self.uh = np.zeros(8)
        rospy.on_shutdown(self.stop)
        self.max_speed = None

    def clamp_qd(self, q_dot: np.ndarray) -> np.ndarray:
        if self.max_speed is None:
            return q_dot
        if (length := np.linalg.norm(q_dot)) > self.max_speed:
            rospy.loginfo_throttle(1, length)
            q_dot = q_dot / length * self.max_speed
        return q_dot

    def uh_cb(self, msg: Float64MultiArray) -> None:
        """human input callback function"""
        self.uh = np.array(msg.data)
        assert len(self.uh) == 8, "human input should be 8 dimensional"
        if self.verbose:
            rospy.loginfo(f"received: {self.uh}")

    @abc.abstractmethod
    def step(self):
        raise NotImplementedError()

    # TODO refactor, function to get both target and ee position
    def get_err(self, target_frame="goal", base_frame="base_link") -> np.ndarray | None:
        try:
            target_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, target_frame, rospy.Time(), timeout=rospy.Duration(secs=5)
            )
            target_pos = target_transform.transform.translation
            ee_transform: TransformStamped = self._tf_buffer.lookup_transform(
                base_frame, "link_grasp_center", rospy.Time(), timeout=rospy.Duration(secs=5)
            )
            ee_pos = ee_transform.transform.translation
            return np.array(
                [
                    target_pos.x - ee_pos.x,
                    target_pos.y - ee_pos.y,
                    target_pos.z - ee_pos.z,
                    0,
                    0,
                    0,
                ]
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            rospy.logerr(e)
            return None

    def publish_vel(self, q_dot: Sequence[float]) -> None:
        self._vel_cmder.pub_vel(q_dot)

    def stop(self) -> None:
        self._vel_cmder.stop()
