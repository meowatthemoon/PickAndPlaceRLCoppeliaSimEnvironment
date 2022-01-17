from typing import List

import numpy as np
from pyrep.objects.vision_sensor import VisionSensor


class Camera:
    def __init__(self, camera_sensor_name: str, resolution: List[int] = None):
        self.camera = VisionSensor(camera_sensor_name)
        if resolution is not None:
            self.camera.set_resolution(resolution)
        self.camera.set_explicit_handling(value=1)

    def get_rgb_depth(self):
        if self.camera is None:
            return None, None

        self.camera.handle_explicitly()

        # RGB
        rgb = self.camera.capture_rgb()
        rgb = np.clip((rgb * 255.).astype(np.uint8), 0, 255)

        # Depth
        depth = self.camera.capture_depth(in_meters=True)  # depth in meters = True
        return rgb, depth

