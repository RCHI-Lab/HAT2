#!/usr/bin/env python3

from __future__ import annotations

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from goal_marker import GoalMarker
from visualization_msgs.msg import Marker

from driver_assistance.msg import GoalBelief, GoalBeliefArray


class MarkerManager:
    def __init__(self, frame="odom") -> None:
        self.frame = frame
        self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.sub = rospy.Subscriber("/goal_beliefs", GoalBeliefArray, self.goal_beliefs_cb)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.goals: dict[int, GoalMarker] = {}
        rospy.on_shutdown(self.goals.clear)

    def goal_beliefs_cb(self, msg: GoalBeliefArray) -> None:
        goal: GoalBelief
        for goal in msg.goals:
            goal_tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.frame, f"goal{goal.id}", rospy.Time()
            )
            goal_pos = goal_tf.transform.translation
            self.add_update_goal(goal.id, (goal_pos.x, goal_pos.y, goal_pos.z), goal.belief)

    def add_update_goal(self, id: int, pos: tuple[float, float, float], belief: float) -> None:
        """Add a new goal marker if it doesn't exist, otherwise update it"""
        if id not in self.goals.keys():
            self.add_goal(id, pos, belief)
        self.update_goal(id, pos, belief)

    def add_goal(self, id: int, pos: tuple[float, float, float], belief: float) -> None:
        """Add a new goal marker"""
        if id in self.goals.keys():
            raise ValueError(f"Goal with id {id} already exists")
        self.goals[id] = GoalMarker(self.pub, *pos, id, self.frame, color=self.belief_color(belief))

    def update_goal(self, id: int, pos: tuple[float, float, float] | None, belief: float | None):
        """Update the position and/or color of a goal marker"""
        if id not in self.goals.keys():
            raise ValueError(f"Goal with id {id} does not exist")
        if belief is None:
            self.goals[id].update(pos=pos)
        else:
            self.goals[id].update(pos=pos, color=self.belief_color(belief))

    def belief_color(
        self, belief: float, opacity: float = 0.8
    ) -> tuple[float, float, float, float]:
        if not 0 <= belief <= 1:
            raise ValueError("Belief must be between 0 and 1")
        return (belief, 0, 0, opacity)


if __name__ == "__main__":
    rospy.init_node("marker_manager")

    manager = MarkerManager()
    rospy.spin()
