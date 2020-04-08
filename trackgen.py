import math
import random

class trackgen:
    def __init__(self):
        self.points = []
        self.robot_pose = None
    
    def gen(self):
        self.robot_pose = (400, 110, 0)
        self.generate_track()
        return self.points

    def generate_track(self):
        self.points = self.get_circle()

    def get_circle(self):
        points = []
        origin = (400, 400)
        r = 240; w = 120; res = 90; current_angle = 0
        while current_angle < 360:
            cx = int(math.cos(math.radians(current_angle)) * r + origin[0])
            cy =  int(math.sin(math.radians(current_angle)) * r + origin[1])
            cxw = int(math.cos(math.radians(current_angle)) * (r + w) + origin[0])
            cyw =  int(math.sin(math.radians(current_angle)) * (r + w) + origin[1])
            points.append((cx, cy))
            points.append((cxw, cyw))
            current_angle += 360 / res
        return points
