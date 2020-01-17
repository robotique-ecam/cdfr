"""Robot tracker base on AR-Tag."""


import cv2
import time
from cv2 import aruco


class Tracker:
    """ARUCO AR TAG Tracker adaptation for robots."""

    def __init__(self):
        """Init Tracker."""
        self.parameters = aruco.DetectorParameters_create()
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)

    def _detect_markers(self, frame):
        """Detect ARUCO marker."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        return corners, ids

    def _regroup_markers(self):
        """Regroup marker corresponding to the same robot."""
        pass

    def _draw(self, gray, corners):
        """Draw markers."""
        return aruco.drawDetectedMarkers(gray, corners)


if __name__ == '__main__':
    from camera import Camera
    cam = Camera()
    tracker = Tracker()
    t = time.time()
    frame = cam.cap.read()[1]
    r = tracker._detect_markers(frame)
    print("Took ", (time.time() - t)*1000, "ms")
    print(r)
    cv2.imwrite("out.png", tracker._draw(frame, r[0]))
