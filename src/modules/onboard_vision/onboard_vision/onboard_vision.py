import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

accuracy = 0.4  # pourcent

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

eceuil_possibility = [
    ['g', 'r', 'g', 'g', 'r'],
    ['g', 'r', 'r', 'g', 'r'],
    ['g', 'g', 'r', 'g', 'r'],
    ['g', 'r', 'g', 'r', 'r'],
    ['g', 'g', 'g', 'r', 'r'],
    ['g', 'g', 'r', 'r', 'r']
]


class OnBoardService(Node):

    def __init__(self):
        super().__init__('onboard_vision')
        self.srv = self.create_service(Trigger, '/get_eceuil_case', self.get_eceuil_case_callback)
        self.srv = self.create_service(Trigger, '/get_north_or_south', self.get_north_or_south_callback)
        self.get_logger().info("onboard_vision node has started")

    def get_eceuil_case_callback(self, request, response):
        response.message = self.get_eceuil_case()
        if response.message != "0":
            response.success = True
        else:
            response.success = False
        return response

    def get_north_or_south_callback(self, request, response):
        side = self.get_north_or_south()
        if side != None:
            self.get_logger().info(side + " end detected")
            response.message = side
            response.success = True
        else:
            self.get_logger().info("no side detected")
            response.message = ""
            response.success = False
        return response

    def get_north_or_south(self):
        cap = cv2.VideoCapture(0)
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        for _ in range(20):
            frame = frame = cv2.flip(cap.read()[1], 0).astype(np.uint8)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            res = cv2.aruco.detectMarkers(gray, dictionary)
            if len(res[0]) > 0:
                cv2.aruco.drawDetectedMarkers(gray, res[0], res[1])
                if res[1][0][0] == 17:
                    result = res[0][0][0][0][0] - res[0][0][0][1][0]
                    if result < 0:
                        return "North"
                    else:
                        return "South"
        return None

    def get_eceuil_case(self):
        cap = cv2.VideoCapture(0)
        for _ in range(20):
            frame = cv2.flip(cap.read()[1], 0).astype(np.uint8)
            im = cv2.blur(frame, (3, 3))
            hsv = cv2.cvtColor(im, cv2.COLOR_BGR2HSV)

            img = im.copy()
            global_mask = np.zeros(im.shape[:-1], np.uint8)

            resultat = {'red': [],
                        'green': []
                        }
            for body in body_colors:
                for lower, upper in body_colors[body]['bounds']:
                    mask = self.color_mask(hsv, lower, upper)
                    global_mask += mask
                    rects = self.min_area_boxes(mask)
                    toDraw = self.accepted(rects)
                    if len(toDraw) != 0:
                        cv2.drawContours(im, toDraw, -1, body_colors[body]['color'], 2)
                        for i in toDraw:
                            split = self.splitGobelet(i)
                            if len(split) == 4:
                                resultat[body].append(self.average(i))
                            else:
                                for j in split:
                                    resultat[body].append(self.average(j))

            if len(resultat['red']) + len(resultat['green']) == 5:
                maybe = []
                for i in resultat['red']:
                    maybe.append(i)
                for i in resultat['green']:
                    maybe.append(i)
                sorted = self.sortList(maybe)
                colors = self.color(sorted, resultat)
                if colors in eceuil_possibility:
                    case = int(eceuil_possibility.index(colors) / 2) + 1
                    cap.release()
                    self.get_logger().info(str(case) + " case ecueil")
                    return str(case)
        return "0"

    def color_mask(self, img, color_lower, color_upper):
        raw_mask = cv2.inRange(img, np.array(color_lower), np.array(color_upper))
        return cv2.morphologyEx(raw_mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    def min_area_boxes(self, mask, threshold=300):
        rects = []
        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            if (cv2.arcLength(contour, True) > threshold):
                rects.append(np.int0(cv2.boxPoints(cv2.minAreaRect(contour))))
        return rects

    def dist2points(self, p1, p2):
        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def accepted(self, rectangle):
        trueRect = []
        for i in rectangle:
            dist = self.dist2points(i[1], i[3])
            dist1 = self.dist2points(i[0], i[2])
            if (((dist <= dist1 * (1 + accuracy / 2)) and (dist >= dist1 * (1 - accuracy / 2))) or
                    ((dist1 <= dist * (1 + accuracy / 2)) and (dist1 >= dist * (1 - accuracy / 2)))):  # and dist1 < dist:
                trueRect.append(i)
        return trueRect

    def getCoords(self, point):
        return point[0], point[1]

    def splitGobelet(self, arr):
        dist = self.dist2points(arr[1], arr[3])
        dist1 = self.dist2points(arr[0], arr[2])
        if dist1 < dist:
            return arr
        x0, y0 = self.getCoords(arr[0])
        x1, y1 = self.getCoords(arr[1])
        x2, y2 = self.getCoords(arr[2])
        x3, y3 = self.getCoords(arr[3])
        if dist < dist1:
            # two gobelets
            two = [[
                [x0, y0], [x1, y1], [(x2 + x0) / 2, (y2 + y0) / 2], [(x3 + x1) / 2, (y3 + y1) / 2]
            ], [
                [(x2 + x0) / 2, (y2 + y0) / 2], [(x3 + x1) / 2, (y3 + y1) / 2], [x2, y2], [x3, y3]
            ]]
            return two
        elif dist < 2.5 * dist1:
            three = [[
                [x0, y0], [x1, y1], [(x2 + x0) / 3, (y2 + y0) / 3], [(x3 + x1) / 3, (y3 + y1) / 3]
            ], [
                [(x2 + x0) / 3, (y2 + y0) / 3], [(x3 + x1) / 3, (y3 + y1) / 3], [2 * (x2 + x0) / 3, 2 * (y2 + y0) / 3], [2 * (x3 + x1) / 3, 2 * (y3 + y1) / 3]
            ], [
                [2 * (x2 + x0) / 3, 2 * (y2 + y0) / 3], [2 * (x3 + x1) / 3, 2 * (y3 + y1) / 3], [x2, y2], [x3, y3]
            ]]
            return three
        return arr

    def average(self, acc):
        x = y = 0
        for i in acc:
            x += i[0]
            y += i[1]
        x = x / 4
        y = y / 4
        return [x, y]

    def sortList(self, toSort):
        sorted = []
        toReturn = []
        for i in toSort:
            sorted.append(i[0])
        sorted.sort()
        for i in sorted:
            for j in toSort:
                if i == j[0]:
                    toReturn.append(j)
        return toReturn

    def color(self, sorted, resultat):
        colors = []
        for i in sorted:
            if i in resultat['red']:
                colors.append('r')
            if i in resultat['green']:
                colors.append('g')
        colors.reverse()
        return colors


def main(args=None):
    rclpy.init(args=args)

    onBoardService = OnBoardService()

    rclpy.spin(onBoardService)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
