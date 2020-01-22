# Standard imports
import cv2
import time
import numpy as np


body_colors = {
    'red': {'bounds': ([0, 0, 110], [25, 55, 255]), 'color': (0, 0, 200)},
    'green': {'bounds': ([0, 50, 0], [50, 95, 55]), 'color': (0, 200, 0)},
}

top_colors = {
    'red':  {'bounds': ([30, 70, 200], [70, 130, 255]), 'color': (0, 0, 255)},
    'green': {'bounds': ([40, 90, 90], [65, 140, 140]),  'color': (0, 255, 0)}
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


def min_enclosing_circle(mask, threshold=300):
    circles = []
    _, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if (cv2.arcLength(contour, True) > threshold):
            (x, y), radius = cv2.minEnclosingCircle(contour)
            center = (int(x), int(y))
            radius = int(radius)
            circles.append([center, radius])
    return circles


# Read image
print("[+] Preparing camera")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)


print("[+] Capturing frame")
im = cv2.blur(cap.read()[1].astype(np.uint8), (3, 3))
img = im.copy()
global_mask = np.zeros(im.shape[:-1], np.uint8)

print("[+] Processing")
t = time.time()

for body in body_colors:
    lower, upper = body_colors[body]['bounds']
    mask = color_mask(im, lower, upper)
    global_mask += mask
    rects = min_area_boxes(mask)
    cv2.drawContours(im, rects, -1, body_colors[body]['color'], 2)

for top in top_colors:
    lower, upper = top_colors[body]['bounds']
    mask = color_mask(im, lower, upper)
    global_mask += mask
    for c in min_enclosing_circle(mask):
        center, radius = c
        cv2.circle(im, center, radius, top_colors[body]['color'], 2)


print("[i] Processing took", (time.time() - t) * 1000, "ms")

cv2.imwrite('contours.png', im)
cv2.imwrite('original.png', img)
cv2.imwrite('detected.png', cv2.bitwise_and(img, img, mask=global_mask))
