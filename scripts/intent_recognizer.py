#!/usr/bin/env python3

from __future__ import annotations

import abc
from math import sqrt

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray

from driver_assistance.msg import GoalBelief, GoalBeliefArray


class IntentRecognizer(abc.ABC):
    def __init__(self, uh_topic: str) -> None:
        self._uh = np.zeros(8)
        self._goal_beliefs: dict[int, float] = {}
        self._gb_pub = rospy.Publisher("/goal_beliefs", GoalBeliefArray, queue_size=1)
        self._uh_sub = rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        self.wait_for_uh(uh_topic)

    def wait_for_uh(self, uh_topic) -> None:
        rospy.loginfo(f"waiting for message from {uh_topic} topic")
        rospy.wait_for_message(uh_topic, Float64MultiArray)

    def uh_cb(self, msg: Float64MultiArray) -> None:
        """human input callback function"""
        self._uh = np.array(msg.data)

    def calculate_beliefs(self, uh) -> None:
        for id in self._goal_beliefs:
            self.calculate_single_belief(id, uh)

    @abc.abstractmethod
    def calculate_single_belief(self, id, uh) -> None:
        """calculate a single goal's belief"""
        pass

    def publish_goal_beliefs(self) -> None:
        self._gb_pub.publish(self._goal_beliefs.items())


class DistanceIR(IntentRecognizer):
    def calculate_single_belief(self, id, uh) -> None:
        if id not in self._goal_beliefs:
            raise ValueError(f"goal id {id} not found")

        pos: Vector3 = self._tf_buffer.lookup_transform(
            "link_grasp_center", f"goal{id}", rospy.Time()
        ).transform.translation
        self._goal_beliefs[id] = sqrt(pos.x**2 + pos.y**2 + pos.z**2)
