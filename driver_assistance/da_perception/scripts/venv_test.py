#!/home/hat/da_ws/src/driver_assistance/da_perception/.venv/bin/python3

import rospy
import torch

rospy.init_node("pytorch_test")
rospy.loginfo("torch version %s", format(torch.__version__))
