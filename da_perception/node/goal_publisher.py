import abc

import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, Transform, TransformStamped, Vector3
from std_msgs.msg import Header, UInt32MultiArray


class GoalPublisherBase(abc.ABC):
    def __init__(self, goal_topic: str, tf_prefix: str) -> None:
        self.goal_topic = goal_topic
        self.goal_pub = rospy.Publisher(goal_topic, UInt32MultiArray, queue_size=2, latch=True)
        self.tf_br = tf2_ros.TransformBroadcaster()
        self.tf_list = []
        self.goal_ids = []
        self.tf_prefix = tf_prefix

    def clear(self):
        self.tf_list.clear()
        self.goal_ids.clear()

    def publish(self):
        self.tf_br.sendTransform(self.tf_list)
        self.goal_pub.publish(UInt32MultiArray(data=self.goal_ids))

    def get_tf(self, detection: dict, timestep, id: int):
        box_3d = detection["box_3d"]
        if box_3d is not None:
            center = box_3d["center_xyz"]
        return TransformStamped(
            header=Header(stamp=timestep, frame_id="camera_color_optical_frame"),
            child_frame_id=f"{self.tf_prefix}{id}",
            transform=Transform(
                translation=Vector3(*center),
                rotation=Quaternion(0, 0, 0, 1),
            ),
        )

    @abc.abstractmethod
    def update(self, detections_3d: list, timestep):
        pass


class SingleGoalPublisher(GoalPublisherBase):
    def __init__(self, goal_topic="/goal_ids", tf_prefix="goal") -> None:
        super().__init__(goal_topic, tf_prefix)

    def update(self, detections_3d: list, timestep):
        """Publish goal_ids and tf for the detection with highest confidence"""
        self.clear()
        if detections_3d:
            detection = max(detections_3d, key=lambda x: x["confidence"])
            self.tf_list.append(self.get_tf(detection, timestep, 0))
            self.goal_ids.append(0)
        self.publish()
