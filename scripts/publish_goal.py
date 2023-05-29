import os
import time


class GzMarker:
    def __init__(self, x, y, z, scale=0.1, id=2) -> None:
        self.x = x
        self.y = y
        self.z = z
        self.scale = scale
        self.id = id
        msg = f"action: ADD_MODIFY, type: SPHERE, id: {self.id}, "
        msg += f"scale: {{x:{self.scale}, y:{self.scale}, z:{self.scale}}}, "
        msg += f"pose: {{position: {{x:{self.x}, y:{self.y}, z:{self.z}}}, orientation: {{x:0, y:0, z:0, w:1}}}}"
        os.system("gz marker -m '" + msg + "'")
        
    def __del__(self) -> None:
        os.system(f"gz marker -m 'action: DELETE_MARKER, id: {self.id}'")

marker = GzMarker(1,1,1)
time.sleep(5)
del marker
