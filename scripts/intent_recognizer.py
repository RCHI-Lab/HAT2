#!/usr/bin/env python3

from __future__ import annotations

import abc
from contextlib import suppress
from math import sqrt

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64MultiArray, UInt32MultiArray

from driver_assistance.msg import GoalBeliefArray


class IntentRecognizer(abc.ABC):
    def __init__(self, perception_topic: str, uh_topic: str | None = None) -> None:
        self._uh = np.zeros(8)
        self._goal_beliefs: dict[int, float] = {}
        self._gb_pub = rospy.Publisher("/goal_beliefs", GoalBeliefArray, queue_size=1)
        self._id_sub = rospy.Subscriber(perception_topic, Float64MultiArray, self.id_cb)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)
        if uh_topic:
            self._uh_sub = rospy.Subscriber(uh_topic, Float64MultiArray, self.uh_cb)
            self.wait_for_uh(uh_topic)

    def wait_for_ids(self, perception_topic) -> None:
        rospy.loginfo(f"waiting for message from {perception_topic} topic")
        rospy.wait_for_message(perception_topic, UInt32MultiArray)
        
    def id_cb(self, msg: UInt32MultiArray) -> None:
        for id in msg.data:
            if id not in self._goal_beliefs:
                self._goal_beliefs[id] = 0
    
    def wait_for_uh(self, uh_topic) -> None:
        rospy.loginfo(f"waiting for message from {uh_topic} topic")
        rospy.wait_for_message(uh_topic, Float64MultiArray)

    def uh_cb(self, msg: Float64MultiArray) -> None:
        """human input callback function"""
        self._uh = np.array(msg.data)

    def calculate_beliefs(self) -> None:
        for id in self._goal_beliefs:
            self.calculate_single_belief(id)

    @abc.abstractmethod
    def calculate_single_belief(self, id) -> None:
        """calculate a single goal's belief"""
        pass

    def publish_goal_beliefs(self) -> None:
        self._gb_pub.publish(GoalBeliefArray(goals=self._goal_beliefs.items()))


class DistanceIR(IntentRecognizer):
    def calculate_single_belief(self, id) -> None:
        if id not in self._goal_beliefs:
            raise ValueError(f"goal id {id} not found")

        pos: Vector3 = self._tf_buffer.lookup_transform(
            "link_grasp_center", f"goal{id}", rospy.Time()
        ).transform.translation
        self._goal_beliefs[id] = sqrt(pos.x**2 + pos.y**2 + pos.z**2)


if __name__ == "__main__":
    rospy.init_node("intent_recognizer")
    ir = DistanceIR("/goal_ids")
    r = rospy.Rate(30)
    with suppress(rospy.ROSInterruptException):
        while not rospy.is_shutdown():
            ir.calculate_beliefs()
            ir.publish_goal_beliefs()
            r.sleep()
