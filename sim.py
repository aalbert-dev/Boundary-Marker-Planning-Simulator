from planner import planner
import random
import math

# simulator class
class simulator:

    # noise in pixels to add to each marker
    def __init__(self, noise=15):
        self.noise = noise
        self.markers = []
        self.robot_start = None

    # add boundary markers to map
    def putMarkers(self, p):
        for x, y in self.markers:
            p.addBoundaryMarker(x, y)
        return p

    # convert pose to distorted pose with noise
    def distortMarker(self, pose_x, pose_y):
        r = random.randint(0, self.noise)
        d = random.randint(0, 360)
        return (int(math.cos(math.radians(d)) * r + pose_x), int(math.sin(math.radians(d)) * r + pose_y))

    # distort and add marker to planner
    def addMarker(self, pose_x, pose_y):
        x, y = self.distortMarker(pose_x, pose_y)
        self.markers.append((x, y))

    # add robot to planner
    def addRobot(self, pose):
        self.robot_start = pose

    #  create new plan object and move robot
    def getplan(self, robot_pose, prev_pts, prev_curve):
        p = planner(previous_midpoints=prev_pts, previous_curve=prev_curve)
        p.addRobotMarker(robot_pose)
        p = self.putMarkers(p)
        p.plan()
        midpoints = prev_pts
        midpoints = p.getMidpoints()
        return (p.getNewPose(), midpoints, p.getCurve())

    # run simulation and display plan
    def simulate(self):

        # plan and move for 150 iterations
        robot_pose = self.robot_start; prev_pts = None; prev_curve = None
        i = 0
        while i < 150:
            robot_pose, pts, curve = self.getplan(robot_pose, prev_pts, prev_curve)
            i += 1
            prev_pts = pts; prev_curve = curve
            

