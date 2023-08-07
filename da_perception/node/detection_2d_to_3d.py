#!/usr/bin/env python3

import numpy as np
from numba_image_to_pointcloud import numba_image_to_pointcloud
from scipy.spatial.transform import Rotation


def filter_points(points_array, camera_matrix, box_2d, min_box_side_m, max_box_side_m):
    # Decompose the camera matrix.
    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    x0, y0, x1, y1 = box_2d
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    z_min = min_box_side_m * min(f_x / detection_box_width_pix, f_y / detection_box_height_pix)
    z_max = max_box_side_m * max(f_x / detection_box_width_pix, f_y / detection_box_height_pix)

    z = points_array[:, 2]
    mask_z = (z > z_min) & (z < z_max)

    # TODO: Handle situations when the cropped rectangle contains no
    # reasonable depth values.

    # Second, filter for depths that are within one maximum head
    # length away from the median depth.
    remaining_z = z[mask_z]
    out_points = np.empty((0, 3), dtype=np.float32)
    if len(remaining_z) > 0:
        median_z = np.median(remaining_z)
        min_z = median_z - max_box_side_m
        max_z = median_z + max_box_side_m
        mask_z = (z > min_z) & (z < max_z)
        remaining_z = z[mask_z]
    if len(remaining_z) > 0:
        out_points = points_array[mask_z]

    return out_points


def bounding_box_2d_to_3d(points_array, box_2d, camera_matrix, head_to_camera_mat=None):
    x0, y0, x1, y1 = box_2d

    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    center_xy_pix = np.array([0.0, 0.0])
    center_xy_pix[0] = (x0 + x1) / 2.0
    center_xy_pix[1] = (y0 + y1) / 2.0
    # These need to be flipped with respect to the basic update
    # function to account for the rotation applied as part of the
    # head orientation estimation.
    detection_box_width_pix = y1 - y0
    detection_box_height_pix = x1 - x0

    num_points = points_array.shape[0]
    if num_points >= 1:
        box_depth = np.median(points_array, axis=0)[2]
    else:
        print(
            "WARNING: No reasonable depth image points available in the detected rectangle. No work around currently implemented for lack of depth estimate."
        )
        return None

    # Convert to 3D point in meters using the camera matrix.
    center_z = box_depth
    center_x = ((center_xy_pix[0] - c_x) / f_x) * center_z
    center_y = ((center_xy_pix[1] - c_y) / f_y) * center_z

    detection_box_width_m = (detection_box_width_pix / f_x) * box_depth
    detection_box_height_m = (detection_box_height_pix / f_y) * box_depth

    if head_to_camera_mat is None:
        R = np.identity(3)
        quaternion = Rotation.from_matrix(R).as_quat()
        x_axis = R[:3, 0]
        y_axis = R[:3, 1]
        z_axis = R[:3, 2]
    else:
        quaternion = Rotation.from_matrix(head_to_camera_mat).as_quat()
        x_axis = head_to_camera_mat[:3, 0]
        y_axis = head_to_camera_mat[:3, 1]
        z_axis = head_to_camera_mat[:3, 2]

    return {
        "center_xyz": (center_x, center_y, center_z),
        "quaternion": quaternion,
        "x_axis": x_axis,
        "y_axis": y_axis,
        "z_axis": z_axis,
        "width_m": detection_box_width_m,
        "height_m": detection_box_height_m,
        "width_pix": detection_box_width_pix,
        "height_pix": detection_box_height_pix,
    }


def detections_2d_to_3d(
    detections_2d,
    rgb_image,
    camera_info,
    depth_image,
    min_box_side_m=None,
    max_box_side_m=None,
):
    orig_h, orig_w, c = rgb_image.shape

    def clip_xy(x_in, y_in):
        x_out = x_in
        y_out = y_in
        x_out = max(0, x_out)
        x_out = min(orig_w - 1, x_out)
        y_out = max(0, y_out)
        y_out = min(orig_h - 1, y_out)
        return x_out, y_out

    camera_matrix = np.reshape(camera_info.K, (3, 3))
    distortion_coefficients = np.array(camera_info.D)

    def clockwise_rotate_bounding_box(box_2d):
        x0, y0, x1, y1 = box_2d
        orig_x0 = (orig_w - 1) - y1
        orig_y0 = x0
        orig_x1 = (orig_w - 1) - y0
        orig_y1 = x1
        return (orig_x0, orig_y0, orig_x1, orig_y1)

    def counterclockwise_rotate_bounding_box(box_2d):
        x0, y0, x1, y1 = box_2d
        orig_x0 = y0
        orig_y0 = (orig_h - 1) - x1
        orig_x1 = y1
        orig_y1 = (orig_h - 1) - x0
        return (orig_x0, orig_y0, orig_x1, orig_y1)

    def clockwise_rotate_xy(x, y):
        return ((orig_w - 1) - y), x

    def counterclockwise_rotate_xy(x, y):
        return y, (orig_h - 1) - x

    rotvec = np.array([0.0, 0.0, 1.0]) * (-np.pi / 2.0)
    counterclockwise_rotate_mat = Rotation.from_rotvec(rotvec).as_matrix()

    detections_3d = []

    for h in detections_2d:
        box_3d = None
        box_2d = h.get("box")
        label = h.get("label")
        points_3d = None

        if box_2d is not None:
            box_2d = counterclockwise_rotate_bounding_box(box_2d)
            x0, y0, x1, y1 = box_2d
            x0, y0 = clip_xy(x0, y0)
            x1, y1 = clip_xy(x1, y1)

            if (
                (x0 < 0)
                or (y0 < 0)
                or (x1 < 0)
                or (y1 < 0)
                or (x0 >= orig_w)
                or (y0 >= orig_h)
                or (x1 >= orig_w)
                or (y1 >= orig_h)
                or (x0 >= x1)
                or (y0 >= y1)
            ):
                print("---------------")
                print(
                    "WARNING: detection bounding box goes outside of the original image dimensions or has other issues, so ignoring detection."
                )
                print("box_2d =", box_2d)
                print("rgb_image.shape =", rgb_image.shape)
                print("---------------")
                box_2d = None

            head_to_camera_mat = counterclockwise_rotate_mat

            if box_2d is not None:
                points_3d = numba_image_to_pointcloud(depth_image, box_2d, camera_matrix)
                if (min_box_side_m is not None) and (max_box_side_m is not None):
                    points_3d = filter_points(
                        points_3d, camera_matrix, box_2d, min_box_side_m, max_box_side_m
                    )
                box_3d = bounding_box_2d_to_3d(
                    points_3d,
                    box_2d,
                    camera_matrix,
                    head_to_camera_mat=head_to_camera_mat,
                )

        detections_3d.append(
            {
                "id": h.get("id"),
                "box_3d": box_3d,
                "box_2d": box_2d,
                "label": label,
                "points_3d": points_3d,
            }
        )

    return detections_3d
