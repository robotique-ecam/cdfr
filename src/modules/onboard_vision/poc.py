# Standard imports
import cv2
import time
import numpy as np


body_colors = {
    'red': {'bounds': [
        ([0, 50, 50], [20, 255, 255]),
        ([170, 50, 50], [180, 255, 255])
    ], 'color': (0, 0, 200)},
    'green': {'bounds': [
        ([34, 50, 50], [80, 255, 255])
    ], 'color': (0, 200, 0)},
}


def color_mask(img, color_lower, color_upper):
    """Return mask of colors in between upper & lower."""
    raw_mask = cv2.inRange(img, np.array(color_lower), np.array(color_upper))
    return cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def min_area_boxes(mask, threshold=120):
    rects = []
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if (cv2.arcLength(contour, True) > threshold):
            rects.append(np.int0(cv2.boxPoints(cv2.minAreaRect(contour))))
    return rects


# Read image
print("[+] Preparing camera")
cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1232)
# cap.set(cv2.CAP_PROP_EXPOSURE,-10)

print("[+] Capturing frame")
frame = cv2.flip(cap.read()[1], 0).astype(np.uint8)
im = cv2.blur(frame, (3, 3))
hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

img = im.copy()
global_mask = np.zeros(im.shape[:-1], np.uint8)

print("[+] Processing")
t = time.time()

for body in body_colors:
    for lower, upper in body_colors[body]['bounds']:
        mask = color_mask(hsv, lower, upper)
        global_mask += mask
        rects = min_area_boxes(mask)
        cv2.drawContours(im, rects, -1, body_colors[body]['color'], 2)


print("[i] Processing took", (time.time() - t) * 1000, "ms")

cv2.imwrite('contours.png', im)
cv2.imwrite('original.png', img)
cv2.imwrite('detected.png', cv2.bitwise_and(img, img, mask=global_mask))
