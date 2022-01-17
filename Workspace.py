from typing import Tuple


class Workspace:
    def __init__(self):
        self._workspace_minx = 0
        self._workspace_maxx = 0.4
        self._workspace_miny = -0.4
        self._workspace_maxy = 0.4
        self._workspace_minz = 0.7519998550415039
        self._workspace_maxz = 1.751999855041504

    def get_bounding_box(self) -> Tuple[float, float, float, float, float, float]:
        return self._workspace_minx, self._workspace_maxx, self._workspace_miny, self._workspace_maxy, self._workspace_minz, self._workspace_maxz
