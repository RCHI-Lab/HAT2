#!/usr/bin/env python3

from __future__ import annotations

import abc
from contextlib import suppress
from math import sqrt

import numpy as np
import rospy
import tf2_ros
from da_core.msg import GoalBelief, GoalBeliefArray
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, UInt32MultiArray


class IntentRecognizer(abc.ABC):
    def __init__(self, perception_topic: str, uh_topic: str | None = None) -> None:
        self._uh = np.zeros(8)
        self._goal_beliefs: dict[int, float | None] = {}
        self._gb_pub = rospy.Publisher("/goal_beliefs", GoalBeliefArray, queue_size=1)
        self._id_sub = rospy.Subscriber(perception_topic, UInt32MultiArray, self.id_cb)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        if uh_topic:
            self._uh_sub = rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
            self.wait_for_uh(uh_topic)
        self.wait_for_ids(perception_topic)

    def wait_for_ids(self, perception_topic: str) -> None:
        rospy.loginfo(f"waiting for message from {perception_topic} topic")
        msg = rospy.wait_for_message(perception_topic, UInt32MultiArray)
        # wait for tfs to be published
        for id in msg.data:
            # TODO use wait for transform
            self._tf_buffer.lookup_transform(
                "link_grasp_center", f"goal{id}", rospy.Time(), rospy.Duration(1)
            )

    def id_cb(self, msg: UInt32MultiArray) -> None:
        for id in msg.data:
            if id not in self._goal_beliefs:
                self._goal_beliefs[id] = 0

    def wait_for_uh(self, uh_topic: str) -> None:
        rospy.loginfo(f"waiting for message from {uh_topic} topic")
        rospy.wait_for_message(uh_topic, Float64MultiArray)

    def uh_cb(self, msg: Float64MultiArray) -> None:
        """human input callback function"""
        self._uh = np.array(msg.data)

    def calculate_beliefs(self) -> None:
        for id in self._goal_beliefs:
            self.calculate_single_belief(id)
            belief = self._goal_beliefs[id]
            assert belief is None or 0 <= belief <= 1

    @abc.abstractmethod
    def calculate_single_belief(self, id: int) -> None:
        """calculate a single goal's belief"""
        pass

    def publish_goal_beliefs(self) -> None:
        self._gb_pub.publish(
            GoalBeliefArray(
                goals=[
                    GoalBelief(id, belief)
                    for id, belief in self._goal_beliefs.items()
                    if belief is not None
                ]
            )
        )

    def get_dist(self, id: int, frame: str) -> float:
        pos: Vector3 = self._tf_buffer.lookup_transform(
            frame, f"goal{id}", rospy.Time()
        ).transform.translation
        return sqrt(pos.x**2 + pos.y**2 + pos.z**2)


class DistanceIR(IntentRecognizer):
    def calculate_single_belief(self, id: int) -> None:
        if id not in self._goal_beliefs:
            raise ValueError(f"goal id {id} not found")

        dist = self.get_dist(id, "link_grasp_center")
        self._goal_beliefs[id] = 1 / (1 + 5 * dist)


class IgnoreFarDeleteGraspIR(IntentRecognizer):
    def calculate_single_belief(self, id: int) -> None:
        if id not in self._goal_beliefs:
            raise ValueError(f"goal id {id} not found")

        dist = self.get_dist(id, "link_grasp_center")
        dist_in_base = self.get_dist(id, "base_link")
        belief = 1 / (1 + 5 * dist)
        self._goal_beliefs[id] = belief if dist_in_base < 1.3 else None


if __name__ == "__main__":
    rospy.init_node("intent_recognizer")
    ir = IgnoreFarDeleteGraspIR("/goal_ids")
    r = rospy.Rate(30)
    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ir.calculate_beliefs()
            rospy.loginfo_throttle(1, ir._goal_beliefs)
            ir.publish_goal_beliefs()
            r.sleep()
