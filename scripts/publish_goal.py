import os
import time

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Quaternion, Pose, Point, Vector3
from std_msgs.msg import Header, ColorRGBA


class GzMarker:
    def __init__(self, pub, x, y, z, scale=0.1, id=2) -> None:
        self.pub = pub
        self.x = x
        self.y = y
        self.z = z
        self.scale = scale
        self.id = id
        msg = f"action: ADD_MODIFY, type: SPHERE, id: {self.id}, "
        msg += f"scale: {{x:{self.scale}, y:{self.scale}, z:{self.scale}}}, "
        msg += f"pose: {{position: {{x:{self.x}, y:{self.y}, z:{self.z}}}, orientation: {{x:0, y:0, z:0, w:1}}}}"
        os.system("gz marker -m '" + msg + "'")
        marker = Marker(
            type=Marker.SPHERE,
            id=self.id,
            lifetime=rospy.Duration(15),
            pose=Pose(Point(self.x, self.y, self.z), Quaternion(0, 0, 0, 1)),
            scale=Vector3(*(self.scale,) * 3),
            header=Header(frame_id="odom"),
            color=ColorRGBA(1.0, 1.0, 1.0, 0.8),
        )
        self.pub.publish(marker)

    def __del__(self) -> None:
        os.system(f"gz marker -m 'action: DELETE_MARKER, id: {self.id}'")
        marker = Marker(action=Marker.DELETE, id=self.id)
        self.pub.publish(marker)


rospy.init_node("rviz_marker")
marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
marker = GzMarker(marker_pub, 1, 1, 1)
time.sleep(10)
del marker
