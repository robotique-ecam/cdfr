"""Points projector."""


import cv2
import numpy as np


class Projector:
    """Undistort toolkit for images and points."""

    def __init__(self, side):
        """Init."""
        self.K = np.array([[400,   0., 600],
                           [0., 400, 360],
                           [0.,   0.,   1.]])
        self.D = np.array([[0.2299233, -0.26828561,  0.00246798, -0.00254312]])
        self.size = (1280, 720)
        self.side = side
        self.Knew = self.K.copy()
        self.points = {
            "final": np.float32([[0, 0], [1280, 0], [0, 720], [1280, 720]]),
            "left": np.float32([[360, 170], [990, 150], [0, 640], [1280, 600]]),
            "right": np.float32([[280, 170], [920, 140], [0, 670], [1280, 600]])
        }
        self._get_transformation_matrix(side)

    def _get_transformation_matrix(self, side):
        """Return transformation matrix."""
        self.M = cv2.getPerspectiveTransform(
            self.points[side], self.points['final'])

    def _perspectiveTransform(self, vector):
        """Perspective transform vector."""
        return cv2.perspectiveTransform(vector, self.M)

    def _perspectiveTransformImage(self, image):
        """Perspective transform an image using the transformation matrix."""
        return cv2.warpPerspective(image, self.M, self.size)

    def _undistort_image(self, img):
        """Undistort an image."""
        return cv2.fisheye.undistortImage(img, self.K, self.D, Knew=self.Knew)

    def _undistort_point(self, points):
        """Undistort point. Must conform to CV_32FC2 or CV_64FC2"""
        return cv2.fisheye.undistortPoints(points, self.K, self.D)


if __name__ == '__main__':
    projector = Projector('right')
    cap = cv2.VideoCapture("/Users/ewen/Desktop/ros/cdfr2k19/src/assurancetourix/data/images/image-right.jpg")
    frame = cap.read()[1]
    cv2.imwrite("out.png", projector._perspectiveTransformImage(frame))
    cv2.imwrite("out2.png", projector._undistort_image(projector._perspectiveTransformImage(frame)))
