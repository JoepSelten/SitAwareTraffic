import numpy as np
import math
import matplotlib.pyplot as plt

from basic_functions import convert_nparray_to_polygon, trans

class Robot():
    def __init__(self, start_pos, start_orient, velocity, length, width, color='green'):
        self.pos = start_pos
        self.yaw = start_orient
        self.length = length
        self.width = width
        self.color = color
        self.velocity = velocity

    def add_resource(self, resource):
        self.resource = resource

    def get_robot_box(self): 
        self.box = trans(self.pos, self.yaw, self.length, self.width)

    def plot_robot(self):
        self.get_robot_box()
        plt.fill(*self.box.exterior.xy, color=self.color)

