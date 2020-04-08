import math
import numpy as np

class linefit:
    def __init__(self, xs, ys, deg):
        self.xs = xs
        self.ys = ys
        self.deg = deg
        self.result = np.polyfit(xs, ys, deg)
    
    def getResult(self):
        return self.result
        
        