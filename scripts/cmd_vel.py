#!/usr/bin/env python3
import os
import time

import rospy
import tf2_ros

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

rospy.init_node("arm_vel_publisher")
arm_vel_pub = rospy.Publisher("stretch_arm_controller/command", Float64MultiArray, queue_size=10)

msg = Float64MultiArray(data=[-x for x in [0.1, 0.025, 0.025, 0.025, 0.025, 1]])
# msg.layout.dim = [MultiArrayDimension(size=0)]
r = rospy.Rate(10)

while not rospy.is_shutdown():
    arm_vel_pub.publish(msg)
    print(msg)
    r.sleep()
