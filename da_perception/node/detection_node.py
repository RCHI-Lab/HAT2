#!/usr/bin/env python3

import struct

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from da_core.msg import GoalBelief, GoalBeliefArray
from detection_2d_to_3d import detections_2d_to_3d
from detection_ros_markers import DetectionBoxMarkerCollection
from goal_publisher import SingleGoalPublisher, NearSingleGoalPublisher
from sensor_msgs import point_cloud2
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from visualization_msgs.msg import MarkerArray


class DetectionNode:
    def __init__(
        self,
        detector,
        default_marker_name,
        node_name,
        topic_base_name,
        min_box_side_m=None,
        max_box_side_m=None,
        modify_3d_detections=None,
    ):
        self.rgb_image = None
        self.depth_image = None
        self.camera_info = None
        self.all_points = []
        self.publish_marker_point_clouds = True

        self.detector = detector

        self.marker_collection = DetectionBoxMarkerCollection(default_marker_name)
        self.topic_base_name = topic_base_name
        self.node_name = node_name
        self.min_box_side_m = min_box_side_m
        self.max_box_side_m = max_box_side_m
        self.modify_3d_detections = modify_3d_detections
        self.image_count = 0
        self.bridge = CvBridge()

    def image_callback(self, ros_rgb_image, ros_depth_image, rgb_camera_info):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(ros_rgb_image, "bgr8")
            self.depth_image = self.bridge.imgmsg_to_cv2(ros_depth_image, "passthrough")
        except CvBridgeError as e:
            print(e)
        self.camera_info = rgb_camera_info
        self.image_count = self.image_count + 1
        self.rgb_image_timestamp = ros_rgb_image.header.stamp
        self.depth_image_timestamp = ros_depth_image.header.stamp

    def update(self):
        if self.rgb_image is None or self.depth_image is None or self.camera_info is None:
            rospy.loginfo_throttle(1, "Waiting for image data...")
            return
        rospy.logdebug(
            f"enter update delay: {(rospy.Time().now() - self.rgb_image_timestamp).to_sec()}"
        )
        rgb_image = self.rgb_image.copy()
        depth_image = self.depth_image.copy()
        camera_info = self.camera_info
        rgb_image_timestamp = self.rgb_image_timestamp
        depth_image_timestamp = self.depth_image_timestamp
        rospy.logdebug(f"after copy: {(rospy.Time().now() - rgb_image_timestamp).to_sec()}")
        # Copy the depth image to avoid a change to the depth image
        # during the update.
        time_diff = rgb_image_timestamp - depth_image_timestamp
        time_diff = abs(time_diff.to_sec())
        if time_diff > 0.0001:
            print("WARNING: The rgb image and the depth image were not taken at the same time.")
            print("         The time difference between their timestamps =", time_diff, "s")

        # Rotate the image by 90deg to account for camera
        # orientation. In the future, this may be performed at the
        # image source.
        detection_box_image = cv2.rotate(rgb_image, cv2.ROTATE_90_CLOCKWISE)

        draw_output = False
        center_crop = True
        detections_2d, output_image = self.detector.apply_to_image(
            detection_box_image, draw_output, crop=center_crop
        )
        for detection in detections_2d:
            rospy.loginfo(f"{detection['label']} detected, score: {detection['confidence']:.3f}")
        rospy.logdebug(f"after 2d: {(rospy.Time().now() - rgb_image_timestamp).to_sec()}")

        detections_3d = detections_2d_to_3d(
            detections_2d,
            rgb_image,
            camera_info,
            depth_image,
            min_box_side_m=self.min_box_side_m,
            max_box_side_m=self.max_box_side_m,
        )
        rospy.logdebug(f"after 3d: {(rospy.Time().now() - rgb_image_timestamp).to_sec()}")

        if self.modify_3d_detections is not None:
            detections_3d = self.modify_3d_detections(detections_3d)

        self.marker_collection.update(detections_3d, rgb_image_timestamp)
        self.goal_publisher.update(detections_3d, rgb_image_timestamp)

        marker_array = self.marker_collection.get_ros_marker_array()
        self.visualize_markers_pub.publish(marker_array)

        self.publish_beliefs(detections_2d)
        rospy.loginfo(f"process delay: {(rospy.Time().now() - rgb_image_timestamp).to_sec()}")

    def add_to_point_cloud(self, x_mat, y_mat, z_mat, mask):
        points = [
            [x, y, z]
            for x, y, z, m in zip(x_mat.flatten(), y_mat.flatten(), z_mat.flatten(), mask.flatten())
            if m > 0
        ]
        self.all_points.extend(points)

    def add_point_array_to_point_cloud(self, point_array):
        if point_array is not None:
            self.all_points.extend(list(point_array))

    def publish_point_cloud(self):
        header = Header()
        header.frame_id = "camera_color_optical_frame"
        header.stamp = rospy.Time.now()
        fields = [
            PointField("x", 0, PointField.FLOAT32, 1),
            PointField("y", 4, PointField.FLOAT32, 1),
            PointField("z", 8, PointField.FLOAT32, 1),
            PointField("rgba", 12, PointField.UINT32, 1),
        ]
        r = 255
        g = 0
        b = 0
        a = 128
        rgba = struct.unpack("I", struct.pack("BBBB", b, g, r, a))[0]
        points = [[x, y, z, rgba] for x, y, z in self.all_points]
        point_cloud = point_cloud2.create_cloud(header, fields, points)
        self.visualize_point_cloud_pub.publish(point_cloud)
        self.all_points = []

    def publish_beliefs(self, detections_2d):
        self.goal_beliefs_pub.publish(
            GoalBeliefArray(
                goals=[
                    GoalBelief(detection["id"], detection["confidence"])
                    for detection in detections_2d
                ]
            )
        )

    def main(self):
        rospy.init_node(self.node_name)
        name = rospy.get_name()
        rospy.loginfo("{0} started".format(name))

        self.goal_publisher = NearSingleGoalPublisher()

        self.rgb_topic_name = "/camera/color/image_raw"  #'/camera/infra1/image_rect_raw'
        self.rgb_image_subscriber = message_filters.Subscriber(self.rgb_topic_name, Image)

        self.depth_topic_name = "/camera/aligned_depth_to_color/image_raw"
        self.depth_image_subscriber = message_filters.Subscriber(self.depth_topic_name, Image)

        self.camera_info_subscriber = message_filters.Subscriber(
            "/camera/color/camera_info", CameraInfo
        )

        self.synchronizer = message_filters.TimeSynchronizer(
            [self.rgb_image_subscriber, self.depth_image_subscriber, self.camera_info_subscriber],
            10,
        )
        self.synchronizer.registerCallback(self.image_callback)

        self.visualize_markers_pub = rospy.Publisher(
            f"/{self.topic_base_name}/marker_array", MarkerArray, queue_size=1
        )
        self.visualize_axes_pub = rospy.Publisher(
            f"/{self.topic_base_name}/axes", MarkerArray, queue_size=1
        )
        self.visualize_point_cloud_pub = rospy.Publisher(
            f"/{self.topic_base_name}/point_cloud2", PointCloud2, queue_size=1
        )

        self.goal_beliefs_pub = rospy.Publisher(
            f"/{self.topic_base_name}/goal_beliefs", GoalBeliefArray, queue_size=1
        )

        rate = rospy.Rate(0.25)
        while not rospy.is_shutdown():
            self.update()
            rate.sleep()
