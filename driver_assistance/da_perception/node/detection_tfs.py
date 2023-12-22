#!/usr/bin/env python3

import tf2_ros
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header


class DetectionTFs:
    """Class to publish the transformation for all detection box"""

    def __init__(self, tf_prefix) -> None:
        self.br = tf2_ros.TransformBroadcaster()
        self.detection_tfs = []
        self.tf_prefix = tf_prefix

    def update(self, detections_3d, timestamp):
        self.detection_tfs.clear()
        for idx, detection in enumerate(detections_3d):
            self.detection_tfs.append(self.get_tf(detection, timestamp, idx))
        self.br.sendTransform(self.detection_tfs)

    def get_tf(self, detection, timestamp, id):
        box_3d = detection["box_3d"]
        if box_3d is not None:
            center = box_3d["center_xyz"]
        return TransformStamped(
            header=Header(stamp=timestamp, frame_id="camera_color_optical_frame"),
            child_frame_id=f"{self.tf_prefix}{id}",
            transform=Transform(
                translation=Vector3(*center),
                rotation=Quaternion(0, 0, 0, 1),
            ),
        )
