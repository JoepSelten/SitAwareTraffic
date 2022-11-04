from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from shapely import affinity
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json
from traffic_areas import *
from basic_functions import *

with open('conf.json') as f:
    config = json.load(f)
    f.close

# with open('map.json') as f:
#     map_dict = json.load(f)
#     f.close

TURTLE_LENGTH = config['turtle_length']
TURTLE_WIDTH = config['turtle_width']
TURTLE_VELOCITY = config['turtle_velocity']
dt = config['dt']
prediction_horizon = config['prediction_horizon']
H = round(prediction_horizon/dt)
b = config['boundary_thickness']
l = config['road_length']
w = config['road_width']
c = config['stop_area_size']


class Simulator():
    def __init__(self):
        self.world = World()
        self.robot = RobotSim('down', TURTLE_LENGTH, TURTLE_WIDTH, 'turtle1')

    def simulate(self):
        self.world.plot_map()
        self.robot.plot_robot()

class World():
    def __init__(self, Traffic_Situation = "Intersection"):
        if Traffic_Situation == "Intersection" :
            self.map_dict =  {'0': {'type': 'on intersection',
                            'geometry': np.array([l, l+w, l+w, l, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'geometry': np.array([l, l+w, l+w, l, 0, 0, l, l]), 'color': 'lightblue', 'transparency': 1},
                        '2': {'type': 'lane', 'location': 'up', 'geometry': np.array([l, l+w, l+w, l, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'lightblue', 'transparency': 1},
                        '3': {'type': 'lane', 'location': 'left', 'geometry': np.array([0, l, l, 0, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'right', 'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l, l, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '5': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '6': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '7': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '8': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '9': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '10': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l+w-0.5*b, l+w-0.5*b, l+w+0.5*b, l+w+0.5*b]), 'color': 'red', 'transparency': 1},
                        '11': {'type': 'boundary', 'location': 'right', 
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '12': {'type': 'boundary', 'location': 'right', 
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l+w-0.5*b, l+w-0.5*b, l+w+0.5*b, l+w+0.5*b]), 'color': 'red', 'transparency': 1},
            }

        elif Traffic_Situation == "Road":

            self.map_dict =  {'0': {'type': 'lane', 'location': 'down', 'direction': [0, -1],
                            'geometry': np.array([l, l+0.5*w, l+0.5*w, l, 0, 0, 2*l, 2*l]), 'color': 'sandybrown', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'direction': [0, -1],
                            'geometry': np.array([l+0.5*w, l+w, l+w, l+0.5*w, 0, 0, 2*l, 2*l]), 'color': 'lightblue', 'transparency': 1},
                        '2': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, 0, 0, 2*l, 2*l]), 'color': 'red', 'transparency': 1},
                        '3': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, 0, 0, 2*l, 2*l]), 'color': 'red', 'transparency': 1},
            }
        
        for key, value in self.map_dict.items():
            self.map_dict[key]['poly'] = convert_nparray_to_polygon(self.map_dict[key]['geometry'])

                
        # for key, value in self.road_dict.items():
        #     self.road_dict[key]['poly'] = convert_nparray_to_polygon(self.road_dict[key]['geometry'])
        
        self.turtles = []
        self.number_of_turtles = 0

    def plot_map(self):
        for key, value in self.map_dict.items():
            plt.fill(*self.map_dict[key]['poly'].exterior.xy, color=self.map_dict[key]['color'], alpha=self.map_dict[key]['transparency'])

    # def plot_road(self):
    #     for key, value in self.road_dict.items():
    #         plt.fill(*self.road_dict[key]['poly'].exterior.xy, color=self.road_dict[key]['color'], alpha=self.road_dict[key]['transparency'])

    def add_robot(self, turtle):
        self.turtles.append(turtle)
        self.number_of_turtles = len(self.turtles)

    def remove_robot(self, turtle_name):
        self.turtles = list(filter(lambda i: i.name != turtle_name, self.turtles))
    
    def print_current_areas(self):
        print(self.turtles[0].current_areas)
    

class RobotSim():
    def __init__(self, start, length, width, name, color='cyan'):
        self.start = start
        self.length = length
        self.width = width
        self.name = name
        self.color = color
        if start == 'down':
            self.pos = [l+0.5*w, self.length/2]
            self.yaw = 0.5*math.pi
            self.lane_id_start = '2'
        elif start == 'up':
            self.pos = [l+0.5*w,2*l+w-self.length/2]
            self.yaw = -0.5*math.pi
            self.lane_id_start = '3'
        elif start == 'left':
            self.pos = [self.length/2,l+0.5*w]
            self.yaw = 0
            self.lane_id_start = '5'
        elif start == 'right':
            self.pos = [2*l+w-self.length/2,l+0.5*w]
            self.yaw = math.pi
            self.lane_id_start = '8'

    def get_robot_box(self): 
        self.box = trans(self.pos, self.yaw, self.length, self.width)

    def plot_robot(self):
        self.get_robot_box()
        plt.fill(*self.box.exterior.xy, color=self.color)