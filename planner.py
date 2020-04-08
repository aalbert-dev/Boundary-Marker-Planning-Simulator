import numpy as np
import cv2 as cv
import math
import random
from intersection import Point, doIntersect
from linefit import linefit

# planner class
class planner:

    # plan correction needs to be True, 
    def __init__(self, previous_midpoints=None, previous_curve=None):
        self.markers = []
        self.robot = None
        self.robot_front = None
        self.orederedMarkers = []
        self.selectedMarkers = []
        self.midpoints = []
        self.all_midpoints = []
        self.points_rejected = 0
        self.show_only_line = True
        self.show_uncorrected = False
        self.show_unselected_markers = True
        self.show_mask = True
        self.show_midpoints = True
        self.left_bound = None
        self.right_bound = None
        self.curve_resolution = 120
        self.show_regression_points = False
        self.mmap = 255 * np.ones(shape=[800, 800, 3], dtype=np.uint8)
        self.curve = None
        self.previous_midpoints = previous_midpoints
        self.previous_curve = previous_curve
        self.goodfit = False
        self.numpts = 0
        self.max_turn = 5
        self.cluster_radius = 40
        self.show_target = True
        self.show_curve = False
        # self.show_target = False
        # self.show_curve = True

    # calls each step of planning process
    def plan(self):
        
        # create empty 1024x1024 image to use as a map
        self.addBoundaryMarkersToMap(self.mmap)
        self.addRobotToMap(self.mmap)
        self.orderMarkers()
        self.selectMarkers(self.mmap)
        self.addAllMidpoints(self.mmap)
        self.addRadialMidpoints(self.mmap)
        self.addRoute(self.mmap)
        self.fitLine(self.mmap)
        self.showMap(self.mmap)

    # return curve coefficents
    def getCurve(self):
        return self.curve

    # return midpoints for a well fitted curve
    def getMidpoints(self):
        return self.midpoints

    # quadratic function generator
    def createQuadFunction(self, coefficents):
        a, b, c = coefficents
        return lambda x: a * x**2 + b * x + c

    # get next position along curve
    def getNewPose(self):

        # needs major improvement
        closest_midpoint = sorted(self.midpoints, key=lambda x: x[0])[len(self.midpoints) // 2]
        denom = float(closest_midpoint[0] -  self.robot_front[0])
        if denom == 0: denom = 1
        #print("D", type(denom), denom)
        numer = float(closest_midpoint[1] - self.robot_front[1])
        #print("N", type(numer), numer)
        #print(str(numer / denom))
        slope = numer / denom
        #print(closest_midpoint[1], self.robot_front[1])
        #print("S", slope)
        z = math.degrees(math.atan(slope)); x, y = self.robot_front; d = self.robot[2]
        print(z)
        if abs(z - d) > self.max_turn:
            if d > z:
                z = d - self.max_turn 
            else:
                z = d + self.max_turn
        return (x, y, z)

    # find left and right bounds of plan
    def findBounds(self, mmap):
        sorted_midpoints = sorted(self.midpoints, key=lambda x: x[0])
        if len(sorted_midpoints) > 5:
            self.left_bound = sorted_midpoints[1][0]
            self.right_bound = sorted_midpoints[-1][0]
        else:
            self.left_bound = 0
            self.right_bound = 800

    # fit line to selected midpoints
    def fitLine(self, mmap):
        x_coordinates = []
        y_coordinates = []
        for x, y in self.midpoints:
            x_coordinates.append(x)
            y_coordinates.append(y)
        l = linefit(x_coordinates, y_coordinates, 2)

        # create lambda expression for function of the form y = ax^2 + bx + c
        f = l.getResult()
        if len(self.midpoints) < 8:
            if self.previous_curve is not None: 
                self.curve = self.previous_curve
                self.midpoints = self.previous_midpoints
                #print("used past curve with " + str(len(self.previous_midpoints)))
                for mp in self.previous_midpoints:
                    cv.circle(mmap, mp, 5, (255, 0, 0))
            else:
                self.curve = f
        else:
            self.curve = f

        # show target midpoint in advance
        if self.show_target:
            closest_midpoint = sorted(self.midpoints, key=lambda x: x[0])[len(self.midpoints) // 2]
            cv.circle(mmap, closest_midpoint, 10, (250, 0, 150), 2)

        a, b, c = self.curve
        self.drawCurve(lambda i: int(a * i**2 + b * i + c), mmap)
    
    # draw curve on map
    def drawCurve(self, f, mmap):
        self.findBounds(mmap)
        curve_pts = []
        x_prev, y_prev = self.robot_front
        step = (self.right_bound - self.left_bound) // self.curve_resolution
        if step == 0: step = 1
        for x in range(self.left_bound, self.right_bound, step):
            y = f(x)
            curve_pts.append((x, y))
            if self.show_curve: cv.line(mmap, (x_prev, y_prev), (x, y), (250, 0, 150), 2)
            x_prev = x; y_prev = y

    # define marker mask
    def getMask(self):
        w = 240; l = 240; t = 120
        p1 = [self.robot_front[0], self.robot_front[1] - w // 2]
        p2 = [self.robot_front[0] + l, self.robot_front[1] - w // 2 - t]
        p3 = [self.robot_front[0] + l, self.robot_front[1] + w // 2 + t]
        p4 = [self.robot_front[0], self.robot_front[1] + w // 2]
        pts = self.rotateMask([p1, p2, p3, p4])
        return np.array(pts, np.int32)

    # rotate mask to match robot pose
    def rotateMask(self, pts):
        rotated_pts = []
        for x, y in pts:
            rotated_pts.append(self.rotatePoint(x, y))
        return rotated_pts

    # rotate point by robot angle
    def rotatePoint(self, x, y):
        x0, y0 = self.robot_front
        d = math.degrees(np.arctan2(x0 - x, y0 - y)) + self.robot[2] + 90
        r = self.distance(self.robot_front, (x, y))
        x1 = int(math.cos(math.radians(d)) * r + x0)
        y1 = int(math.sin(math.radians(d)) * r + y0)
        return [x1, y1]

    # check if a given point is in poly mask
    def pointInMask(self, x, y, pts):
        point_right_line = ((x, y), (x + 1000, y))
        mask_line_1 = (pts[0], pts[1])
        mask_line_2 = (pts[1], pts[2])
        mask_line_3 = (pts[2], pts[3])
        mask_line_4 = (pts[3], pts[0])
        num_intersections = 0
        if self.intersects(point_right_line, mask_line_1):
            num_intersections += 1
        if self.intersects(point_right_line, mask_line_2):
            num_intersections += 1
        if self.intersects(point_right_line, mask_line_3):
            num_intersections += 1
        if self.intersects(point_right_line, mask_line_4):
            num_intersections += 1
        return num_intersections % 2 == 1

    # checks if two lines defined by points intersect
    def intersects(self, l1, l2):
        p1 = Point(l1[0][0], l1[0][1])
        q1 = Point(l1[1][0], l1[1][1])
        p2 = Point(l2[0][0], l2[0][1])
        q2 = Point(l2[1][0], l2[1][1])
        return doIntersect(p1, q1, p2, q2)

    # select markers to use to calculate midpoints
    def selectMarkers(self, mmap):
        pts = self.getMask()
        if self.show_mask: cv.polylines(mmap, [pts], True, (0, 0, 0), 1)
        for i in range(0, len(self.orederedMarkers)):
            if i < len(self.orederedMarkers):
                x, y, z = self.orederedMarkers[i]
                if not self.pointInMask(x, y, pts):
                    if not self.show_unselected_markers: 
                        cv.circle(mmap, (x,y), 2, (255, 255, 255), 3) 
                    else:
                        cv.circle(mmap, (x,y), 2, (120, 120, 120), 3) 
                else:
                    self.selectedMarkers.append((x, y))

    # add route to map
    def addRoute(self, mmap):
        
        # iterate for each midpoint to use for curve 
        prev_point = (self.robot_front[0], self.robot_front[1])
        for i in range(0, len(self.midpoints)):
            cur_point = self.midpoints[i]
            if self.show_regression_points: cv.line(mmap, prev_point, cur_point, (150, 150, 0), 2)
            prev_point = cur_point

    # order markers by distance to robot
    def orderMarkers(self):
        r_x = self.robot[0]
        r_y = self.robot[1]
        orderedMarkers = []
        for x, y in self.markers:
            d = int(self.distance((r_x, r_y), (x, y)))
            orderedMarkers.append((x, y, d))
        orderedMarkers.sort(key=lambda x: x[2], reverse=False)
        self.orederedMarkers = orderedMarkers

    def addAllMidpoints(self, mmap):
        # add each mid point to all midpoints
        prev_marker = self.selectedMarkers[0]
        for i in range(1, len(self.selectedMarkers)):
            cur_marker = self.selectedMarkers[i]
            if not self.show_only_line:
                cv.line(mmap, (prev_marker[0], prev_marker[1]), (cur_marker[0], cur_marker[1]), (0, 255, 0), 2)
            midpoint = ((prev_marker[0] + cur_marker[0]) // 2, (prev_marker[1] + cur_marker[1]) // 2)
            self.all_midpoints.append(midpoint)
            if self.show_midpoints: cv.circle(mmap, midpoint, 2, (0, 165, 255), 3) 
            prev_marker = cur_marker

    # add mid points from triangles using radial distance function
    def addRadialMidpoints(self, mmap):

        # cluster and draw midpoints for curve 
        self.midpoints = self.clusterPoints()
        if self.show_midpoints: 
            for midpoint in self.midpoints: 
                cv.circle(mmap, midpoint, 2, (0, 255, 0), 3) 
        
        # number of points to use for curve
        self.numpts = len(self.midpoints)

    def clusterPoints(self):

        # midpoints to try to clister from
        points_to_consider = self.all_midpoints

        # first point to cluster from
        start_point = self.robot_front
    
        # hold midpoints in order of most recently added
        final_points = []

        # add starting point 
        final_points.append(start_point)

        
        while True:

            while True:
                if not points_to_consider: break

                # sort points by distance to last point in final points in decreasing order
                points_to_consider.sort(key=lambda x: self.distance(x, final_points[-1]), reverse=False)

                # if the distance is less than a threshold otherwise break
                if self.distance(final_points[-1], points_to_consider[0]) < self.cluster_radius:

                    # add point to final points
                    final_points.append(points_to_consider[0])

                    # remove from points to consider
                    points_to_consider.pop(0)
                else:
                    break

            if len(final_points) > 8:
                break
            else:
                if len(self.all_midpoints) > 0:
                    start_point = self.all_midpoints[random.randint(0, len(self.all_midpoints) - 1)]
                else:
                    break

                final_points = []
                final_points.append(start_point)
                points_to_consider = self.all_midpoints


        return final_points
            

    # draw distance from robot to each marker
    def drawDistances(self, mmap):
        r_x = self.robot[0]
        r_y = self.robot[1]
        for x, y in self.markers:
            d = int(self.distance((r_x, r_y), (x, y)))
            cv.putText(mmap, str(d), (x, y), cv.FONT_HERSHEY_SIMPLEX , 0.5, (0, 0, 0))

    # add markers to map
    def addBoundaryMarkersToMap(self, mmap):
        for x, y in self.markers:
            cv.circle(mmap, (x,y), 2, (0, 0, 255), 3) 

    # add robot vector to map
    def addRobotToMap(self, mmap):
        r = 10
        r_x = self.robot[0]
        r_y = self.robot[1]
        d = self.robot[2]
        r_xp = int(math.cos(math.radians(d)) * r + r_x)
        r_yp =  int(math.sin(math.radians(d)) * r + r_y)
        self.robot_front = (r_xp, r_yp)
        cv.line(mmap, (r_x, r_y), (r_xp, r_yp), (255, 0, 0), 2)
        cv.circle(mmap, (r_xp, r_yp), 1, (0, 255, 0), 1)

    # distance function for two points
    def distance(self, p1, p2):
        return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

    # show map with opencv 
    def showMap(self, img):
        cv.imshow("Map", img)
        cv.waitKey(0)
        cv.destroyAllWindows()

    # add boundary marker to data structure
    def addBoundaryMarker(self, x, y):
        self.markers.append((x, y))

    # add robot marker to data structure
    def addRobotMarker(self, pose):
        self.robot = pose