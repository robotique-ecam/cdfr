# Standard imports
import cv2
import time
import numpy as np

accuracy = 0.4 #pourcent

body_colors = {
    'red': {'bounds': [
        ([0, 110, 110], [10, 255, 255]),
        ([170, 50, 50], [180, 255, 255])
    ], 'color': (0, 0, 200)},
    'green': {'bounds': [
        ([50, 0, 50], [255, 10, 255]),
        ([23, 50, 23], [255, 255, 255])
    ], 'color': (0, 250, 0)},
}

eceuil_possibility =[
['g','r','g','g','r'],
['g','r','r','g','r'],
['g','g','r','g','r'],
['g','r','g','r','r'],
['g','g','g','r','r'],
['g','g','r','r','r']
]


def color_mask(img, color_lower, color_upper):
    """Return mask of colors in between upper & lower."""
    raw_mask = cv2.inRange(img, np.array(color_lower), np.array(color_upper))
    return cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def min_area_boxes(mask, threshold=300):
    rects = []
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    for contour in contours:
        if (cv2.arcLength(contour, True) > threshold):
            rects.append(np.int0(cv2.boxPoints(cv2.minAreaRect(contour))))
    return rects

def accepted(rectangle):
    trueRect = []
    for i in rectangle:
        dist = abs(i[1][0]-i[3][0])+abs(i[1][1]-i[3][1])
        dist1 = abs(i[0][0]-i[2][0])+abs(i[0][1]-i[2][1])
        if (((dist <= dist1*(1+accuracy/2)) and (dist >= dist1*(1-accuracy/2))) or \
        ((dist1 <= dist*(1+accuracy/2)) and (dist1 >= dist*(1-accuracy/2)))) and dist1 < dist:
            trueRect.append(i)
    return trueRect

def average(acc):
    x = y = 0
    for i in acc:
        x += i[0]
        y += i[1]
    x = x/4
    y = y/4
    return [x,y]

def sortList(toSort):
    sorted = []
    toReturn=[]
    for i in toSort:
        sorted.append(i[0])
    sorted.sort()
    for i in sorted:
        for j in toSort:
            if i == j[0]:
                toReturn.append(j)
    return toReturn

def color(sorted, resultat):
    colors = []
    for i in sorted:
        if i in resultat['red']:
            colors.append('r')
        if i in resultat['green']:
            colors.append('g')
    colors.reverse()
    return colors

# Read image
print('[+] Preparing camera')
cap = cv2.VideoCapture(0)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1640)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1232)
# cap.set(cv2.CAP_PROP_EXPOSURE,-10)
while (1):
    #print('[+] Capturing frame')
    frame = cv2.flip(cap.read()[1], 0).astype(np.uint8)
    #frame = cv2.imread("original1.png")
    im = cv2.blur(frame, (3, 3))
    hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

    img = im.copy()
    global_mask = np.zeros(im.shape[:-1], np.uint8)

    t = time.time()
    resultat = {'red': [],
                'green': []
                }
    for body in body_colors:
        for lower, upper in body_colors[body]['bounds']:
            mask = color_mask(hsv, lower, upper)
            global_mask += mask
            rects = min_area_boxes(mask)
            toDraw = accepted(rects)
            if len(toDraw) != 0:
                for i in toDraw:
                    resultat[body].append(average(i))
                cv2.drawContours(im, toDraw, -1, body_colors[body]['color'], 2)

    if len(resultat['red']) + len(resultat['green']) == 5:
        maybe = []
        for i in resultat['red']:
            maybe.append(i)
        for i in resultat['green']:
            maybe.append(i)
        sorted = sortList(maybe)
        colors = color(sorted, resultat)
        if colors in eceuil_possibility:
            case = int(eceuil_possibility.index(colors)/2) + 1
            print(case, end ="\n")

    cv2.imshow("annotated", im)
    cv2.waitKey(3)

#cv2.imwrite('contours.png', im)
#cv2.imwrite('original.png', img)
#cv2.imwrite('detected.png', cv2.bitwise_and(img, img, mask=global_mask))
