from __future__ import annotations
import abc

import rospy
import tf2_ros
from geometry_msgs.msg import Point, Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header, UInt32MultiArray
from tf2_geometry_msgs import PointStamped


class GoalPublisherBase(abc.ABC):
    def __init__(self, goal_topic: str, tf_prefix: str) -> None:
        self.goal_topic = goal_topic
        self.goal_pub = rospy.Publisher(goal_topic, UInt32MultiArray, queue_size=2, latch=True)
        self.tf_br = tf2_ros.StaticTransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.tf_list: list[TransformStamped] = []
        self.goal_ids: list[int] = []
        self.tf_prefix = tf_prefix

    def clear(self):
        self.tf_list.clear()
        self.goal_ids.clear()

    def publish(self):
        self.tf_br.sendTransform(self.tf_list)
        self.goal_pub.publish(UInt32MultiArray(data=self.goal_ids))

    def get_tf(self, detection: dict, timestamp, id: int):
        assert (rospy.Time().now() - timestamp).to_sec() < 10
        box_3d = detection["box_3d"]
        if box_3d is None:
            # return immediately because ros messages has default values if arg is None
            return None
        center = box_3d["center_xyz"]
        point_camera = PointStamped(
            header=Header(stamp=timestamp, frame_id="camera_color_optical_frame"),
            point=Point(x=center[0], y=center[1], z=center[2]),
        )
        point_odom: PointStamped = self.tf_buffer.transform_full(
            point_camera, "odom", timestamp, "odom"
        )
        return TransformStamped(
            header=Header(stamp=timestamp, frame_id="odom"),
            child_frame_id=f"{self.tf_prefix}{id}",
            transform=Transform(
                translation=Vector3(point_odom.point.x, point_odom.point.y, point_odom.point.z),
                rotation=Quaternion(0, 0, 0, 1),
            ),
        )

    @abc.abstractmethod
    def update(self, detections_3d: list, timestamp):
        raise NotImplementedError()


class SingleGoalPublisher(GoalPublisherBase):
    def __init__(self, goal_topic="/goal_ids", tf_prefix="goal") -> None:
        super().__init__(goal_topic, tf_prefix)

    def update(self, detections_3d: list, timestamp):
        """Publish goal_ids and tf for the detection with highest confidence"""
        self.clear()
        if detections_3d:
            detection = max(detections_3d, key=lambda x: x["confidence"])
            tf = self.get_tf(detection, timestamp, 0)
            if tf is None:
                return
            self.tf_list.append(tf)
            self.goal_ids.append(0)
        self.publish()
