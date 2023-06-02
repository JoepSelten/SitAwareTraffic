from basic_areas import *
import math
import numpy as np

class Lane():
    side_width = 1
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0]), color='lightblue'):
        forward_pos = pos + rel_coord
        self.forward_area = Forward(pos, yaw, length, width, rel_coord, color)
        side_rel_coord = np.array([0, -0.5*width]) + rel_coord
        side_of_lane_pos = pos + side_rel_coord
        self.side_of_lane = NoEnter(pos, yaw, length, self.side_width, side_rel_coord)
        self.box = self.forward_area.box.union(self.side_of_lane.box)

    def plot_area(self):
        self.forward_area.plot_area()
        self.side_of_lane.plot_area()

class OneLaneRoad():
    side_width = 1
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0])):
        self.lane = Lane(pos, yaw, length, width)
        side_rel_coord = np.array([0, 0.5*width])
        self.other_side_of_lane = NoEnter(pos, yaw, length, self.side_width, side_rel_coord)
        self.box = self.lane.box.union(self.other_side_of_lane.box)
        
    def plot_area(self):
        self.lane.plot_area()
        self.other_side_of_lane.plot_area()
       
class TwoLaneRoad():
    def __init__(self, pos, yaw, length, width, rel_coord=np.array([0,0])):
        lane_rel_coord = np.array([0, -0.25*width]) + rel_coord
        self.left_lane = Lane(pos, yaw, length, 0.5*width, lane_rel_coord, 'sandybrown')
        self.right_lane = Lane(pos, -1*yaw, length, 0.5*width, lane_rel_coord)
        self.box = self.left_lane.box.union(self.right_lane.box)
        
    def plot_area(self):
        self.left_lane.plot_area()
        self.right_lane.plot_area()
        

class Intersection():
    def __init__(self, pos, length, width, rel_coord=np.array([0,0])):
        south_pos = pos + np.array([0, -0.5*length-0.5*width])
        east_pos = pos + np.array([0.5*length+0.5*width, 0])
        west_pos = pos + np.array([-0.5*length-0.5*width, 0])
        north_pos = pos + np.array([0, 0.5*length+0.5*width])
        self.south_road = TwoLaneRoad(south_pos, length, width, np.array([1, 0]))
        self.east_road = TwoLaneRoad(east_pos, length, width, np.array([0, -1]))
        self.west_road = TwoLaneRoad(west_pos, length, width, np.array([0, 1]))
        self.north_road = TwoLaneRoad(north_pos, length, width, np.array([-1, 0]))

    def plot_area(self):
        self.south_road.plot_area()
        self.east_road.plot_area()
        self.west_road.plot_area()
        self.north_road.plot_area()