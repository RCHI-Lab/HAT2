#!/usr/bin/env python3

"""
Automatically publish goal marker in gazebo, rviz and tf server
author: Chen Chen
"""

import os

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Pose, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import ColorRGBA, Header
from visualization_msgs.msg import Marker


class GzMarker:
    def __init__(self, pub, x, y, z, scale=0.1, id=2) -> None:
        self.pub = pub
        self.x = x
        self.y = y
        self.z = z
        self.scale = scale
        self.id = id
        # gazebo marker
        msg = f"action: ADD_MODIFY, type: SPHERE, id: {self.id}, "
        msg += f"scale: {{x:{self.scale}, y:{self.scale}, z:{self.scale}}}, "
        msg += f"pose: {{position: {{x:{self.x}, y:{self.y}, z:{self.z}}}, orientation: {{x:0, y:0, z:0, w:1}}}}"
        os.system("gz marker -m '" + msg + "'")
        # rviz marker
        marker = Marker(
            type=Marker.SPHERE,
            id=self.id,
            pose=Pose(Point(self.x, self.y, self.z), Quaternion(0, 0, 0, 1)),
            scale=Vector3(*(self.scale,) * 3),
            header=Header(frame_id="odom"),
            color=ColorRGBA(1.0, 1.0, 1.0, 0.8),
        )
        self.pub.publish(marker)
        # tf broadcast
        self.br = tf2_ros.StaticTransformBroadcaster()
        tf_stamped = TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id="odom"),
            child_frame_id="ee_goal",
            transform=Transform(
                translation=Vector3(self.x, self.y, self.z), rotation=Quaternion(0, 0, 0, 1)
            ),
        )
        self.br.sendTransform(tf_stamped)

    def __del__(self) -> None:
        os.system(f"gz marker -m 'action: DELETE_MARKER, id: {self.id}'")
        marker = Marker(action=Marker.DELETE, id=self.id)
        self.pub.publish(marker)


if __name__ == "__main__":
    rospy.init_node("gazebo_marker")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    marker = GzMarker(marker_pub, 0, 0, 0.5)

    rospy.on_shutdown(marker.__del__)

    rate = rospy.Rate(1)

    try:
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
