#!/usr/bin/env python3

"""
Automatically publish goal marker in gazebo, rviz and tf server
author: Chen Chen
"""

from __future__ import annotations

import os

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class GoalMarker:
    def __init__(
        self,
        pub: rospy.Publisher,
        x: float,
        y: float,
        z: float,
        id: int,
        frame: str,
        scale: float = 0.1,
        color: tuple[float, float, float, float] = (1.0, 1.0, 1.0, 0.8),
        tf_prefix: str = "goal",
    ) -> None:
        self.pub = pub
        self.x = x
        self.y = y
        self.z = z
        self.id = id
        self.frame = frame
        self.scale = scale
        self.color = color
        self.tf_prefix = tf_prefix
        self.br = tf2_ros.StaticTransformBroadcaster()
        self.publish_marker()

    def publish_marker(self):
        self.publish_gazebo_marker()
        self.publish_rviz_marker()
        self.broadcast_tf()

    def broadcast_tf(self):
        tf_stamped = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=self.frame),
            child_frame_id=f"{self.tf_prefix}{self.id}",
            transform=Transform(
                translation=Vector3(self.x, self.y, self.z), rotation=Quaternion(0, 0, 0, 1)
            ),
        )
        self.br.sendTransform(tf_stamped)

    def publish_rviz_marker(self):
        marker = Marker(
            header=Header(stamp=rospy.Time.now(), frame_id=self.frame),
            id=self.id,
            type=Marker.SPHERE,
            action=Marker.MODIFY,
            pose=Pose(Point(self.x, self.y, self.z), Quaternion(0, 0, 0, 1)),
            scale=Vector3(*(self.scale,) * 3),
            color=ColorRGBA(*self.color),
        )
        self.pub.publish(marker)

    def publish_gazebo_marker(self):
        msg = f"action: ADD_MODIFY, type: SPHERE, id: {self.id}, "
        msg += f"scale: {{x:{self.scale}, y:{self.scale}, z:{self.scale}}}, "
        msg += f"pose: {{position: {{x:{self.x}, y:{self.y}, z:{self.z}}}, orientation: {{x:0, y:0, z:0, w:1}}}}"
        os.system("gz marker -m '" + msg + "'")

    @property
    def position(self) -> tuple[float, float, float]:
        return (self.x, self.y, self.z)

    @position.setter
    def position(self, value: tuple[float, float, float]):
        if len(value) != 3:
            raise AttributeError("position is not 3 dimentional")
        self.x, self.y, self.z = value

    def update(
        self,
        pos: tuple[float, float, float] | None = None,
        color: tuple[float, float, float, float] | None = None,
    ):
        if pos is not None:
            self.position = pos
        if color is not None:
            self.color = color
        self.publish_marker()

    def __del__(self) -> None:
        # cannot delete tf frame
        os.system(f"gz marker -m 'action: DELETE_MARKER, id: {self.id}'")
        marker = Marker(action=Marker.DELETE, id=self.id)
        self.pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("gazebo_marker")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    marker = GoalMarker(marker_pub, *(1, 1, 1), id=1, frame="odom")
    rospy.on_shutdown(marker.__del__)

    rospy.sleep(1)

    marker.update(pos=(0, 1, 2), color=(0, 1, 0, 0.8))

    rospy.spin()
