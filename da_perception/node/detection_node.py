#!/usr/bin/env python3

import struct

import cv2
import message_filters
import rospy
from cv_bridge import CvBridge, CvBridgeError
from da_core.msg import GoalBelief, GoalBeliefArray
from detection_2d_to_3d import detections_2d_to_3d
from detection_ros_markers import DetectionBoxMarkerCollection
from detection_tfs import DetectionTFs
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
        self.rgb_image_timestamp = None
        self.depth_image = None
        self.depth_image_timestamp = None
        self.camera_info = None
        self.all_points = []
        self.publish_marker_point_clouds = True

        self.detector = detector

        self.marker_collection = DetectionBoxMarkerCollection(default_marker_name)

        self.detection_tfs = DetectionTFs(tf_prefix=default_marker_name)

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
        self.rgb_image_timestamp = ros_rgb_image.header.stamp
        self.depth_image_timestamp = ros_depth_image.header.stamp
        self.camera_info = rgb_camera_info
        self.image_count = self.image_count + 1

        # Copy the depth image to avoid a change to the depth image
        # during the update.
        time_diff = self.rgb_image_timestamp - self.depth_image_timestamp
        time_diff = abs(time_diff.to_sec())
        if time_diff > 0.0001:
            print("WARNING: The rgb image and the depth image were not taken at the same time.")
            print("         The time difference between their timestamps =", time_diff, "s")

        # Rotate the image by 90deg to account for camera
        # orientation. In the future, this may be performed at the
        # image source.
        detection_box_image = cv2.rotate(self.rgb_image, cv2.ROTATE_90_CLOCKWISE)

        debug_input = False
        if debug_input:
            print("DetectionNode.image_callback: received an image!")
            print(
                "DetectionNode.image_callback: detection_box_image.shape =",
                detection_box_image.shape,
            )
            cv2.imwrite(
                "./output_images/deep_learning_input_" + str(self.image_count).zfill(4) + ".png",
                detection_box_image,
            )

        debug_output = False
        center_crop = True
        detections_2d, output_image = self.detector.apply_to_image(
            detection_box_image, draw_output=debug_output, crop=center_crop
        )
        for detection in detections_2d:
            rospy.logdebug(f"{detection['label']} detected, score: {detection['confidence']:.3f}")

        if debug_output:
            print("DetectionNode.image_callback: processed image with deep network!")
            print("DetectionNode.image_callback: output_image.shape =", output_image.shape)
            cv2.imwrite(
                "./output_images/deep_learning_output_" + str(self.image_count).zfill(4) + ".png",
                output_image,
            )

        detections_3d = detections_2d_to_3d(
            detections_2d,
            self.rgb_image,
            self.camera_info,
            self.depth_image,
            min_box_side_m=self.min_box_side_m,
            max_box_side_m=self.max_box_side_m,
        )

        if self.modify_3d_detections is not None:
            detections_3d = self.modify_3d_detections(detections_3d)

        self.marker_collection.update(detections_3d, self.rgb_image_timestamp)
        self.detection_tfs.update(detections_3d, self.rgb_image_timestamp)

        marker_array = self.marker_collection.get_ros_marker_array()
        include_axes = True
        include_z_axes = False
        axes_array = None
        axes_scale = 4.0
        if include_axes or include_z_axes:
            axes_array = self.marker_collection.get_ros_axes_array(
                include_z_axes, include_axes, axes_scale=axes_scale
            )

        if self.publish_marker_point_clouds:
            for marker in self.marker_collection:
                marker_points = marker.get_marker_point_cloud()
                self.add_point_array_to_point_cloud(marker_points)
            self.publish_point_cloud()
        self.visualize_markers_pub.publish(marker_array)
        if axes_array is not None:
            self.visualize_axes_pub.publish(axes_array)

        self.publish_beliefs(detections_2d)

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
