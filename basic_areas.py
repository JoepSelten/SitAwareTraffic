from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from shapely import affinity
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json

from basic_functions import convert_nparray_to_polygon, trans

class RectangleArea():
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0]), color='lightblue'):
        self.pos = pos
        self.length = length
        self.width = width
        self.yaw = yaw
        self.color = color
        self.rel_coord = rel_coord
        self.box = trans(self.pos, self.yaw, self.length, self.width, self.rel_coord)

    def plot_area(self):
            plt.fill(*self.box.exterior.xy, color=self.color)

class Forward(RectangleArea):
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0]), color='lightblue'):
        super().__init__(pos, yaw, length, width, rel_coord, color)
        self.yaw = yaw
        self.task = "Move forward"

class NoEnter(RectangleArea):
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0]), color='red'):
        super().__init__(pos, yaw, length, width, rel_coord, color)
        self.task = "No Enter"

class Avoid(RectangleArea):
    def __init__(self, pos, length, width, rel_coord=np.array([0,0]), color='sandybrown'):
        super().__init__(pos, length, width, rel_coord, color)
        self.task = "Avoid"