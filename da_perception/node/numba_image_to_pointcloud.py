import numpy as np
from numba import njit


@njit(fastmath=True)
def numba_image_to_pointcloud(depth_image, bounding_box, camera_matrix):
    x_min, y_min, x_max, y_max = bounding_box
    h, w = depth_image.shape

    # check and correct the bounding box to be within the rgb_image
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

    out_w = (x_max - x_min) + 1
    out_h = (y_max - y_min) + 1
    points = np.empty((out_h * out_w, 3), dtype=np.float32)

    i = 0
    x = x_min
    while x <= x_max:
        y = y_min
        while y <= y_max:
            z_3d = depth_image[y, x] / 1000.0
            x_3d = ((x - c_x) / f_x) * z_3d
            y_3d = ((y - c_y) / f_y) * z_3d
            points[i] = (x_3d, y_3d, z_3d)
            i += 1
            y += 1
        x += 1

    return points
