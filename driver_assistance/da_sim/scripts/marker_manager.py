#!/usr/bin/env python3

from __future__ import annotations

import rospy
from goal_marker import GoalMarker
from visualization_msgs.msg import Marker

from da_core.msg import GoalBelief, GoalBeliefArray


class MarkerManager:
    def __init__(self, prefix="goal") -> None:
        self.prefix = prefix
        self.pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.sub = rospy.Subscriber("/goal_beliefs", GoalBeliefArray, self.goal_beliefs_cb)
        self.goals: dict[int, GoalMarker] = {}
        rospy.on_shutdown(self.goals.clear)

    def goal_beliefs_cb(self, msg: GoalBeliefArray) -> None:
        goal: GoalBelief
        for goal in msg.goals:
            self.add_update_goal(goal.id, goal.belief)

    def add_update_goal(self, id: int, belief: float) -> None:
        """Add a new goal marker if it doesn't exist, otherwise update it"""
        if id not in self.goals.keys():
            self.add_goal(id, belief)
        else:
            self.update_goal(id, belief)

    def add_goal(self, id: int, belief: float) -> None:
        """Add a new goal marker"""
        if id in self.goals.keys():
            raise ValueError(f"Goal with id {id} already exists")
        self.goals[id] = GoalMarker(
            self.pub, 0, 0, 0, id, f"{self.prefix}{id}", color=self.belief_color(belief)
        )

    def update_goal(self, id: int, belief: float):
        """Update the position and/or color of a goal marker"""
        if id not in self.goals.keys():
            raise ValueError(f"Goal with id {id} does not exist")
        self.goals[id].update(color=self.belief_color(belief))

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
