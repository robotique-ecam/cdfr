"""Camera class."""


import cv2
from PIL import Image


class Camera:
    """Burst camera for in-game image capture."""

    def __init__(self, input=0):
        """Init camera with input camera id."""
        self.cap = cv2.VideoCapture(input)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.cap.read()

    def capture(self):
        """Capture a frame."""
        img = self.cap.read()[1]
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        return Image.fromarray(img)
