#!/usr/bin/env python3

import sys

import cv2
import detection_node as dn
import rospy
import owlvit_detector as od

if __name__ == "__main__":
    print("cv2.__version__ =", cv2.__version__)
    print("Python version (must be > 3.0):", sys.version)
    assert int(sys.version[0]) >= 3

    confidence_threshold = 0.08

    detector = od.OwlViTObjectDetector(score_threshold=confidence_threshold)
    default_marker_name = "object"
    node_name = "DetectObjectsNode"
    topic_base_name = "objects"
    node = dn.DetectionNode(detector, default_marker_name, node_name, topic_base_name)
    node.main()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("interrupt received, so shutting down")
