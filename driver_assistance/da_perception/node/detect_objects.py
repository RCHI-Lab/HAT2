#!/usr/bin/env python3

import sys

import cv2
import rospy
from detection_node import DetectionNode
from owlvit_detector import OwlViTObjectDetector

if __name__ == "__main__":
    print("cv2.__version__ =", cv2.__version__)
    print("Python version (must be > 3.0):", sys.version)
    assert int(sys.version[0]) >= 3

    queries = rospy.get_param("queries", ["black tumbler"])
    print("queries =", queries)
    confidence_threshold = 0.08
    detector = OwlViTObjectDetector(queries, score_threshold=confidence_threshold)

    default_marker_name = "detection"
    node_name = "DetectObjectsNode"
    topic_base_name = "detections"
    node = DetectionNode(detector, default_marker_name, node_name, topic_base_name)
    node.main()
