#!/usr/bin/env python3

from __future__ import annotations

import random
from math import sqrt

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header, UInt32MultiArray


def dist_between_tfs(buffer: tf2_ros.Buffer, tf1: str, tf2: str) -> float | None:
    try:
        target_transform: TransformStamped = buffer.lookup_transform(
            tf1, tf2, rospy.Time(), timeout=rospy.Duration(secs=5)
        )
        pos = target_transform.transform.translation
        return sqrt(pos.x**2 + pos.y**2 + pos.z**2)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        rospy.logerr(e)
        return None


def get_new_marker_pose(buffer: tf2_ros.Buffer, range=0.4) -> tuple[float, float, float]:
    target_transform: TransformStamped = buffer.lookup_transform(
        "odom", "base_link", rospy.Time(), timeout=rospy.Duration(secs=5)
    )
    pos = target_transform.transform.translation
    x = random.choice((-1, 1)) * random.uniform(0.2, range)
    y = random.choice((-1, 1)) * random.uniform(0.2, range)
    z = random.uniform(0.1, 1)
    return (x + pos.x, y + pos.y, z + pos.z)


if __name__ == "__main__":
    rospy.init_node("goal_manager")
    pub = rospy.Publisher("/goal_ids", UInt32MultiArray, queue_size=2, latch=True)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    tf_br = tf2_ros.StaticTransformBroadcaster()
    frame = "odom"

    goals: dict[int, tuple[float, float, float]] = {
        i: get_new_marker_pose(tf_buffer) for i in range(5)
    }
    rospy.on_shutdown(goals.clear)

    # send a tf list because of the latching mechanism (can only send last message to new subscribers)
    tf_list = [
        TransformStamped(
            header=Header(stamp=rospy.Time.now(), frame_id=frame),
            child_frame_id=f"goal{id}",
            transform=Transform(translation=Vector3(*goals[id]), rotation=Quaternion(0, 0, 0, 1)),
        )
        for id in goals
    ]
    tf_br.sendTransform(tf_list)
    pub.publish(UInt32MultiArray(data=goals.keys()))

    rospy.spin()
