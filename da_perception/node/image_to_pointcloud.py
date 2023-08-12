import numpy as np


def image_to_pointcloud(depth_image, bounding_box, camera_matrix):
    x_min, y_min, x_max, y_max = bounding_box
    h, w = depth_image.shape[:2]
    x_min = int(round(max(0, x_min)))
    y_min = int(round(max(0, y_min)))
    x_max = int(round(min(w - 1, x_max)))
    y_max = int(round(min(h - 1, y_max)))
    x_max = max(x_max, x_min)
    y_max = max(y_max, y_min)
    f_x = camera_matrix[0, 0]
    c_x = camera_matrix[0, 2]
    f_y = camera_matrix[1, 1]
    c_y = camera_matrix[1, 2]

    x, y = np.meshgrid(np.arange(x_min, x_max + 1), np.arange(y_min, y_max + 1))
    z_3d = depth_image[y, x] / 1000.0
    x_3d = ((x - c_x) / f_x) * z_3d
    y_3d = ((y - c_y) / f_y) * z_3d
    return np.column_stack((x_3d.ravel(), y_3d.ravel(), z_3d.ravel()))
