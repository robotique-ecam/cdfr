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
        ([35, 50, 35], [255, 255, 255])
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

def dist2points(p1, p2):
    return abs(p1[0]-p2[0])+abs(p1[1]-p2[1])

def accepted(rectangle):
    trueRect = []
    for i in rectangle:
        dist = dist2points(i[1], i[3])
        dist1 = dist2points(i[0], i[2])
        if (((dist <= dist1*(1+accuracy/2)) and (dist >= dist1*(1-accuracy/2))) or \
        ((dist1 <= dist*(1+accuracy/2)) and (dist1 >= dist*(1-accuracy/2)))): #and dist1 < dist:
            trueRect.append(i)
    return trueRect

def getCoords(point):
    return point[0], point[1]

def splitGobelet(arr):
    dist = dist2points(arr[1], arr[3])
    dist1 = dist2points(arr[0], arr[2])
    if dist1 < dist:
        return arr
    x0, y0 = getCoords(arr[0])
    x1, y1 = getCoords(arr[1])
    x2, y2 = getCoords(arr[2])
    x3, y3 = getCoords(arr[3])
    if dist < dist1:
        #two gobelets
        two = [[
        [x0,y0],[x1,y1],[(x2+x0)/2, (y2+y0)/2],[(x3+x1)/2, (y3+y1)/2]
        ],[
        [(x2+x0)/2, (y2+y0)/2],[(x3+x1)/2, (y3+y1)/2],[x2,y2],[x3,y3]
        ]]
        return two
    elif dist < 2.5 * dist1:
        three = [[
        [x0,y0],[x1,y1],[(x2+x0)/3, (y2+y0)/3],[(x3+x1)/3, (y3+y1)/3]
        ],[
        [(x2+x0)/3, (y2+y0)/3], [(x3+x1)/3, (y3+y1)/3], [2*(x2+x0)/3, 2*(y2+y0)/3], [2*(x3+x1)/3, 2*(y3+y1)/3]
        ],[
        [2*(x2+x0)/3, 2*(y2+y0)/3], [2*(x3+x1)/3, 2*(y3+y1)/3],[x2,y2],[x3,y3]
        ]]
        return three
    return arr



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
                cv2.drawContours(im, toDraw, -1, body_colors[body]['color'], 2)
                for i in toDraw:
                    split = splitGobelet(i)
                    if len(split) == 4:
                        resultat[body].append(average(i))
                    else:
                        for j in split:
                            resultat[body].append(average(j))

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

    cv2.imshow("colors", cv2.bitwise_and(img, img, mask=global_mask))
    cv2.imshow("around", im)
    cv2.waitKey(3)

#cv2.imwrite('contours.png', im)
#cv2.imwrite('original.png', img)
#cv2.imwrite('detected.png', cv2.bitwise_and(img, img, mask=global_mask))
