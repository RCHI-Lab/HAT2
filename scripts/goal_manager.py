#!/usr/bin/env python3

import random
from math import sqrt

import rospy
import tf2_ros
from gazebo_marker import GazeboMarker
from geometry_msgs.msg import TransformStamped
from tf2_ros import Buffer
from visualization_msgs.msg import Marker


def dist_between_tfs(buffer: Buffer, tf1: str, tf2: str):
    try:
        target_transform: TransformStamped = buffer.lookup_transform(tf1, tf2, rospy.Time())
        pos = target_transform.transform.translation
        return sqrt(pos.x**2 + pos.y**2 + pos.z**2)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        rospy.logerr(e)
        return None


def get_new_marker_pose(buffer, range=0.4):
    try:
        target_transform: TransformStamped = buffer.lookup_transform(
            "odom", "base_link", rospy.Time()
        )
        pos = target_transform.transform.translation
        x = random.choice((-1, 1)) * random.uniform(0.2, range)
        y = random.choice((-1, 1)) * random.uniform(0.2, range)
        z = random.uniform(0.1, 1)
        return (x + pos.x, y + pos.y, z + pos.z)
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        rospy.logerr(e)
        return None


if __name__ == "__main__":
    rospy.init_node("goal_manager")
    marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.sleep(1)
    # marker = GazeboMarker(marker_pub, 1, 1, 1)
    marker_list = [
        GazeboMarker(marker_pub, *get_new_marker_pose(tf_buffer)),
    ]

    rospy.on_shutdown(marker_list.clear)

    rate = rospy.Rate(5)

    try:
        while not rospy.is_shutdown():
            if (dist := dist_between_tfs(tf_buffer, "ee_goal", "link_grasp_center")) is not None:
                rospy.loginfo_throttle(0.5, f"distance: {dist}")
                if dist < 0.01:
                    marker_list.clear()
                    marker_list.append(GazeboMarker(marker_pub, *get_new_marker_pose(tf_buffer)))
                    rospy.sleep(0.5)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
