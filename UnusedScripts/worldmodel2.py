from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
from shapely import affinity
import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.pyplot import figure
import json

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

d = config['intersection_monitor_size']
R_left = 0.75*w
R_right = 0.25*w
K_angle = 5
D_angle = 0.5
K_vel = 2
D_vel = 1
K_alpha = 0.04
alpha_max = 2
omega_max = 1
T1 = omega_max/alpha_max


def convert_nparray_to_polygon(poly_ndarray):
        array_tmp = poly_ndarray.squeeze()
        x = array_tmp[0:4]
        y = array_tmp[4:8]
        polygon = Polygon([(x[i], y[i]) for i in range(0, 4)])
        return polygon

class WorldModel():
    def __init__(self, Traffic_Situation = "Intersection"):

        if Traffic_Situation == "Intersection" :
            self.map_dict =  {'0': {'type': 'on intersection',
                            'geometry': np.array([l, l+w, l+w, l, l, l, l+w, l+w]), 'color': 'silver', 'transparency': 1},
                        '1': {'type': 'lane', 'location': 'down', 'direction': [0, -1],
                            'geometry': np.array([l, l+0.5*w, l+0.5*w, l, 0, 0, l, l]), 'color': 'sandybrown', 'transparency': 1},
                        '2': {'type': 'lane', 'location': 'down', 'direction': [0, 1],
                            'geometry': np.array([l+0.5*w, l+w, l+w, l+0.5*w, 0, 0, l, l]), 'color': 'lightblue', 'transparency': 1},   
                        '3': {'type': 'lane', 'location': 'up', 'direction': [0, -1],
                            'geometry': np.array([l, l+0.5*w, l+0.5*w, l, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'lightblue', 'transparency': 1},
                        '4': {'type': 'lane', 'location': 'up', 'direction': [0, 1],
                            'geometry': np.array([l+0.5*w, l+w, l+w, l+0.5*w, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'sandybrown', 'transparency': 1},
                        '5': {'type': 'lane', 'location': 'left', 'direction': [1, 0],
                            'geometry': np.array([0, l, l, 0, l, l, l+0.5*w, l+0.5*w]), 'color': 'lightblue', 'transparency': 1},
                        '6': {'type': 'lane', 'location': 'left', 'direction': [-1, 0],
                            'geometry': np.array([0, l, l, 0, l+0.5*w, l+0.5*w, l+w, l+w]), 'color': 'sandybrown', 'transparency': 1},
                        '7': {'type': 'lane', 'location': 'right', 'direction': [1, 0],
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l, l, l+0.5*w, l+0.5*w]), 'color': 'sandybrown', 'transparency': 1},
                        '8': {'type': 'lane', 'location': 'right', 'direction': [-1, 0],
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l+0.5*w, l+0.5*w, l+w, l+w]), 'color': 'lightblue', 'transparency': 1},
                        '9': {'type': 'at intersection',
                            'geometry': np.array([l-d, l+w+d, l+w+d, l-d, l-d, l-d, l+w+d, l+w+d]), 'color': 'yellow', 'transparency': 0.15},
                        '10': {'type': 'stop_area', 'location': 'down',
                            'geometry': np.array([l+0.5*w, l+w, l+w, l+0.5*w, l-c, l-c, l, l]), 'color': 'lime', 'transparency': 1},
                        '11': {'type': 'stop_area', 'location': 'up',
                            'geometry': np.array([l, l+0.5*w, l+0.5*w, l, l+w, l+w, l+w+c, l+w+c]), 'color': 'lime', 'transparency': 1},
                        '12': {'type': 'stop_area', 'location': 'left',
                            'geometry': np.array([l-c, l, l, l-c, l, l, l+0.5*w, l+0.5*w]), 'color': 'lime', 'transparency': 1},
                        '13': {'type': 'stop_area', 'location': 'right',
                            'geometry': np.array([l+w, l+w+c, l+w+c, l+w, l+0.5*w, l+0.5*w, l+w, l+w]), 'color': 'lime', 'transparency': 1}, 
                        '14': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '15': {'type': 'boundary', 'location': 'down', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, 0, 0, l, l]), 'color': 'red', 'transparency': 1},
                        '16': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l-0.5*b, l+0.5*b, l+0.5*b, l-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '17': {'type': 'boundary', 'location': 'up', 
                            'geometry': np.array([l+w-0.5*b, l+w+0.5*b, l+w+0.5*b, l+w-0.5*b, l+w, l+w, 2*l+w, 2*l+w]), 'color': 'red', 'transparency': 1},
                        '18': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '19': {'type': 'boundary', 'location': 'left', 
                            'geometry': np.array([0, l, l, 0, l+w-0.5*b, l+w-0.5*b, l+w+0.5*b, l+w+0.5*b]), 'color': 'red', 'transparency': 1},
                        '20': {'type': 'boundary', 'location': 'right', 
                            'geometry': np.array([l+w, 2*l+w, 2*l+w, l+w, l-0.5*b, l-0.5*b, l+0.5*b, l+0.5*b]), 'color': 'red', 'transparency': 1},
                        '21': {'type': 'boundary', 'location': 'right', 
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


class Robot():
    def __init__(self, start, goal, length, width, name, world, color='cyan'):
        self.start = start
        self.length = length
        self.rob_area = length*width
        self.name = name
        self.world = world
        self.goal = goal
        if self.goal == 'down':
            self.lane_id_goal = '1'
        elif self.goal == 'up':
            self.lane_id_goal = '4'
        elif self.goal == 'left':
            self.lane_id_goal = '6'
        elif self.goal == 'right':
            self.lane_id_goal = '7'

        if start == 'down':
            self.pos = [l+0.75*w, self.length/2]
            self.yaw = 0.5*math.pi
            self.lane_id_start = '2'
        elif start == 'up':
            self.pos = [l+0.25*w,2*l+w-self.length/2]
            self.yaw = -0.5*math.pi
            self.lane_id_start = '3'
        elif start == 'left':
            self.pos = [self.length/2,l+0.25*w]
            self.yaw = 0
            self.lane_id_start = '5'
        elif start == 'right':
            self.pos = [2*l+w-self.length/2,l+0.75*w]
            self.yaw = math.pi
            self.lane_id_start = '8'
        
        self.width = width
        self.current_point = 0
        self.color = color
        self.finished = False
        self.current_areas = {'on intersection': 0, 'lane': 0, 'at intersection': 0, 'stop_area': 0}
        self.pred_areas = {'on intersection': 0, 'lane': 0, 'at intersection': 0, 'stop_area': 0}
        self.omega = 0
        self.set_relative_yaw = False
        self.yaw_before_intersect = 0
        self.current_direction = 0
        self.pred_direction = 0
        self.prev_error = 0
        self.situation = 'Idle'
        self.velocity = TURTLE_VELOCITY
        self.acc = 0
        self.jerk = 0
        self.stop = 0
        self.alpha = 0
        self.prev_error_vel = 0
        self.give_priority = False
        self.pred_prev_yaw_error = 0
        self.in_stop_area = False
        self.pred_in_stop_area = False
        self.pred_prev_vel_error = 0

        self.set_turn_timer = False
        self.t_current = 0

        self.action_finished = np.zeros(H-1)
        self.turning =  np.zeros(H-1, dtype=bool)
        self.time_turning = np.zeros(H-1)
 
    def get_pos(self):
        return self.pos

    def get_orientation(self):
        return self.yaw

    def plot_constraints(self):
        for constraint in self.no_enter_constraints:
            plt.fill(*constraint.exterior.xy, color='indigo')
        for constraint in self.avoid_constraints:
            plt.fill(*constraint.exterior.xy, color='indigo', alpha=0.5)

    def get_constraints(self):              # later liever wat slimmer doen
        self.no_enter_constraints = []
        self.avoid_constraints = []
        if self.semantic_position == 'lane':
            if self.lane_id == '2':
                self.avoid_constraints.append(self.world.map_dict['1']['poly'])    # lane next to
                self.no_enter_constraints.append(self.world.map_dict['14']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['15']['poly'])   # boundaries
            if self.lane_id == '1':
                self.avoid_constraints.append(self.world.map_dict['2']['poly'])   
                self.no_enter_constraints.append(self.world.map_dict['14']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['15']['poly'])  
            if self.lane_id == '3':
                self.avoid_constraints.append(self.world.map_dict['4']['poly'])    
                self.no_enter_constraints.append(self.world.map_dict['16']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['17']['poly']) 
            if self.lane_id == '4':
                self.avoid_constraints.append(self.world.map_dict['3']['poly'])   
                self.no_enter_constraints.append(self.world.map_dict['16']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['17']['poly'])  
            if self.lane_id == '5':
                self.avoid_constraints.append(self.world.map_dict['6']['poly'])   
                self.no_enter_constraints.append(self.world.map_dict['18']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['19']['poly'])  
            if self.lane_id == '6':
                self.avoid_constraints.append(self.world.map_dict['5']['poly'])    
                self.no_enter_constraints.append(self.world.map_dict['18']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['19']['poly'])   
            if self.lane_id == '7':
                self.avoid_constraints.append(self.world.map_dict['8']['poly'])    
                self.no_enter_constraints.append(self.world.map_dict['20']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['21']['poly'])  
            if self.lane_id == '8':
                self.avoid_constraints.append(self.world.map_dict['7']['poly'])   
                self.no_enter_constraints.append(self.world.map_dict['20']['poly'])
                self.no_enter_constraints.append(self.world.map_dict['21']['poly'])
        if self.semantic_position == 'intersection':
                if self.lane_id_goal == '6':
                    self.avoid_constraints.append(self.world.map_dict['1']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['2']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['3']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['4']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['5']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['7']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['8']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['16']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['19']['poly'])
                if self.lane_id_goal == '1':
                    self.avoid_constraints.append(self.world.map_dict['2']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['3']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['4']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['5']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['6']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['7']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['8']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['14']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['18']['poly'])
                if self.lane_id_goal == '4':
                    self.avoid_constraints.append(self.world.map_dict['1']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['2']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['3']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['5']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['6']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['7']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['8']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['17']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['21']['poly'])
                if self.lane_id_goal == '7':
                    self.avoid_constraints.append(self.world.map_dict['1']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['2']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['3']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['4']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['5']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['6']['poly'])
                    self.avoid_constraints.append(self.world.map_dict['8']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['15']['poly'])
                    self.no_enter_constraints.append(self.world.map_dict['20']['poly'])



    def create_goal(self, goal):
        self.goal = goal
        if goal == 'down':
            self.goal_pos = [45,self.length/2]
            self.goal_yaw = -0.5*math.pi
            self.after_intersect = [45,40]
            if self.start == 'left':
                self.goal_yaw = -0.5*math.pi
        elif goal == 'up':
            self.goal_pos = [55,100-self.length/2]
            self.goal_yaw = 0.5*math.pi
            self.after_intersect = [55,60]
        elif goal == 'left':
            self.goal_pos = [self.length/2,55]
            self.goal_yaw = math.pi
            self.after_intersect = [40,55]
        elif goal == 'right':
            self.goal_pos = [100-self.length/2,45]
            self.goal_yaw = 0
            self.after_intersect = [60,45]
            if self.start == 'up':
                self.goal_yaw = 2*math.pi

    def move_robot(self, trajectory):
        self.current_point += 1
        x, y, yaw = trajectory
        
        if len(x) > self.current_point:
            self.pos[0] = x[self.current_point] 
            self.pos[1] = y[self.current_point]
            self.yaw = yaw[self.current_point]
        else:
            self.finished = True

    def move_robot2(self, velocity, dt):
        yaw_prev = self.yaw
        x_prev = self.pos[0]
        y_prev = self.pos[1]
        if self.current_areas['on intersection'] > self.current_areas['lane']:
            if (self.start == 'down' and self.goal == 'left') or (self.start == 'up' and self.goal == 'right') \
        or (self.start == 'left' and self.goal == 'up') or (self.start == 'right' and self.goal == 'down'):
                self.yaw = yaw_prev + math.acos(1-0.5*(velocity*dt/R_left)**2)
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
            elif (self.start == 'down' and self.goal == 'right') or (self.start == 'up' and self.goal == 'left') \
        or (self.start == 'left' and self.goal == 'down') or (self.start == 'right' and self.goal == 'up'):
                self.yaw = yaw_prev - math.acos(1-0.5*(velocity*dt/R_right)**2)
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
            else:
                self.yaw = yaw_prev
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
        else:
            self.yaw = yaw_prev
            self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
            self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt

        if sum(self.current_areas.values()) < 0.99*self.rob_area:
            self.finished = True

    def move_robot3(self, dt, alpha):
        yaw_prev = self.yaw
        omega_prev = self.omega
        x_prev = self.pos[0]
        y_prev = self.pos[1]
        if self.current_areas['on intersection'] > self.current_areas['lane']:
            if (self.start == 'down' and self.goal == 'left') or (self.start == 'up' and self.goal == 'right') \
        or (self.start == 'left' and self.goal == 'up') or (self.start == 'right' and self.goal == 'down'):
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True
                x = R_left
                self.alpha = ((velocity**2)*math.pi)/(2*(x**2))
                if self.yaw-self.yaw_before_intersect > 0.25*math.pi:
                    self.alpha = -self.alpha
                self.omega = omega_prev + self.alpha*dt
                self.yaw = yaw_prev + self.omega*dt
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
            elif (self.start == 'down' and self.goal == 'right') or (self.start == 'up' and self.goal == 'left') \
        or (self.start == 'left' and self.goal == 'down') or (self.start == 'right' and self.goal == 'up'):
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True
                x = R_left
                self.alpha = -((velocity**2)*math.pi)/(2*(x**2))
                if self.yaw-self.yaw_before_intersect < -0.25*math.pi:
                    self.alpha = -self.alpha
                self.omega = omega_prev + self.alpha*dt
                self.yaw = yaw_prev + self.omega*dt
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
            else:
                self.yaw = yaw_prev
                self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt
        else:
            self.yaw = yaw_prev
            self.pos[0] = x_prev + velocity*math.cos(self.yaw)*dt
            self.pos[1] = y_prev + velocity*math.sin(self.yaw)*dt

        if sum(self.current_areas.values()) < 0.99*self.rob_area:
            self.finished = True

    def move_robot4(self, dt, alpha):
        yaw_prev = self.yaw
        omega_prev = self.omega
        vel_prev = self.velocity
        acc_prev = self.acc
        x_prev = self.pos[0]
        y_prev = self.pos[1]
        self.alpha = 0
        self.jerk = 0
        self.check_sit()
        action = self.return_action()

        if action == 'slow down':
            stop_dist = 10
            if self.stop < stop_dist/2:
                self.jerk = -(pow(self.velocity,3)/pow(stop_dist,2))
            else:
                self.jerk = (pow(self.velocity,3)/pow(stop_dist,2))
            self.acc = acc_prev + self.jerk*dt
            self.velocity = vel_prev + self.acc*dt
            self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
            self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
            self.stop += self.velocity*math.sin(self.yaw)*dt
            return     

        if action == 'stop': 
            velocity_error = 0 - self.velocity
            self.jerk = 1*velocity_error
            self.acc = acc_prev + self.jerk*dt
            self.velocity = vel_prev + self.acc*dt
            self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
            self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
            return

        if self.current_areas['on intersection'] > self.current_areas['lane']:
            if (self.start == 'down' and self.goal == 'left') or (self.start == 'up' and self.goal == 'right') \
        or (self.start == 'left' and self.goal == 'up') or (self.start == 'right' and self.goal == 'down'):
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True
                x = R_left

                velocity_error = 15 - self.velocity
                
                self.jerk = 3*velocity_error + 100*(velocity_error-self.prev_error_vel)
                self.acc = acc_prev + self.jerk*dt
                self.velocity = vel_prev + self.acc*dt
                self.prev_error_vel = velocity_error

                self.alpha = ((self.velocity**2)*math.pi)/(2*(x**2))
                if self.yaw-self.yaw_before_intersect > 0.25*math.pi:
                    self.alpha = -self.alpha
                self.omega = omega_prev + self.alpha*dt
                self.yaw = yaw_prev + self.omega*dt

                self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
                self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
            elif (self.start == 'down' and self.goal == 'right') or (self.start == 'up' and self.goal == 'left') \
        or (self.start == 'left' and self.goal == 'down') or (self.start == 'right' and self.goal == 'up'):
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True
                x = R_right

                velocity_error = 15 - self.velocity
                
                self.jerk = 3*velocity_error + 100*(velocity_error-self.prev_error_vel)
                self.acc = acc_prev + self.jerk*dt
                self.velocity = vel_prev + self.acc*dt
                self.prev_error_vel = velocity_error

                self.alpha = -((self.velocity**2)*math.pi)/(2*(x**2))
                if self.yaw-self.yaw_before_intersect < -0.25*math.pi:
                    self.alpha = -self.alpha
                self.omega = omega_prev + self.alpha*dt
                self.yaw = yaw_prev + self.omega*dt
            else:
                velocity_error = 15 - self.velocity
                self.jerk = 3*velocity_error + 100*(velocity_error-self.prev_error_vel)
                self.acc = acc_prev + self.jerk*dt
                self.velocity = vel_prev + self.acc*dt
                self.prev_error_vel = velocity_error
                self.yaw = yaw_prev
        else:
            self.yaw = yaw_prev

        if self.current_areas['lane'] > 0.5*self.rob_area:
            yaw_error = math.atan2(self.current_direction[1], self.current_direction[0]) - self.yaw
            if yaw_error > math.pi:
                yaw_error -= 2*math.pi
            if yaw_error < -math.pi:
                yaw_error += 2*math.pi

            velocity_error = 15 - self.velocity

            self.jerk = 3*velocity_error + 10*(velocity_error-self.prev_error_vel)
            self.acc = acc_prev + self.jerk*dt
            self.velocity = vel_prev + self.acc*dt
            self.prev_error_vel = velocity_error


            self.alpha = 10*yaw_error + 50*(yaw_error-self.prev_error)
            self.omega += self.alpha*dt
            self.yaw += self.omega*dt
            self.prev_error = yaw_error
        
        self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
        self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt

        if sum(self.current_areas.values()) < 0.99*self.rob_area:
            self.finished = True

    def move(self):
        self.get_semantic_position()
        #self.get_constraints()
        self.check_sit()
        self.return_action()
        self.MPC()
        if sum(self.current_areas.values()) < 0.9*self.rob_area:
            self.finished = True

    def MPC(self):
        self.predict2()              # feedforward and plot
        self.control()                # feedback if not
        
    def predict(self):
        # yaw_prev = self.yaw
        # omega_prev = self.omega
        # vel_prev = self.velocity
        # acc_prev = self.acc
        # x_prev = self.pos[0]
        # y_prev = self.pos[1]
        self.x_pred = np.zeros(H)
        self.y_pred = np.zeros(H)
        self.yaw_pred = np.zeros(H)
        self.acc_pred = np.zeros(H)
        self.velocity_pred = np.zeros(H)
        omega_pred = np.zeros(H)
        self.x_pred[0] = self.pos[0]
        self.y_pred[0] = self.pos[1]
        self.yaw_pred[0] = self.yaw
        self.acc_pred[0] = self.acc
        self.velocity_pred[0] = self.velocity
        omega_pred[0] = self.omega
        self.alpha_pred = np.zeros(H-1)
        self.jerk_pred = np.zeros(H-1)

        self.alpha_feedback = 0
        self.stop = 0
        stop_dist = d/2
        
        self.pred_semantic_position = self.semantic_position
        self.pred_in_stop_area = self.in_stop_area
        self.future_robot_area = self.robot_area
        self.pred_direction = self.current_direction
        self.pred_prev_yaw_error = 0
        velocity_error = TURTLE_VELOCITY - self.velocity

        self.no_enter_constraints_satisfied = False
        self.avoid_constraints_satisfied = False
       
        while not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
            self.point_not_satisfied = 0
        #print(self.action)
            if self.action == 'drive straight':          # deze hoeft eigenlijk niet, dit moet ie gwn altijd doen als ie in de lane is
                for i in range(1,H):
                    self.x_pred[i] = self.x_pred[i-1] + self.velocity*math.cos(self.yaw)*dt
                    self.y_pred[i] = self.y_pred[i-1] + self.velocity*math.sin(self.yaw)*dt
                self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                self.get_future_semantic_position()
                #self.plot_constraints()
                #plt.plot(self.x_pred, self.y_pred, 'r--')
                #plt.waitforbuttonpress()
                

            if self.action == 'turn left':
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True
                
                for i in range(1,H):
                    if self.pred_semantic_position == 'intersection':
                        #print(self.world.map_dict[self.lane_id_goal]['geometry'])
                        if self.yaw_pred[i-1]-self.yaw_before_intersect <= 0.25*math.pi:
                            self.alpha_pred[i-1] = ((self.velocity**2)*math.pi)/(2*(R_left**2))
                        elif self.yaw_pred[i-1]-self.yaw_before_intersect > 0.25*math.pi and self.yaw_pred[i-1] \
                        - self.yaw_before_intersect < 0.5*math.pi:
                            self.alpha_pred[i-1] = -((self.velocity**2)*math.pi)/(2*(R_left**2))
                        elif self.yaw_pred[i-1]-self.yaw_before_intersect >= 0.5*math.pi:
                            self.alpha_pred[i-1] = 0
                        self.alpha_pred[i-1] += self.alpha_feedback
                    elif self.pred_semantic_position == 'lane':
                        if self.alpha_feedback == 0:
                            yaw_error = math.atan2(self.pred_direction[1], self.pred_direction[0]) - self.yaw_pred[i-1]
                            if yaw_error > math.pi:
                                yaw_error -= 2*math.pi
                            if yaw_error < -math.pi:
                                yaw_error += 2*math.pi
                            #self.alpha_pred[i-1] = K_angle*yaw_error + D_angle*(yaw_error-self.pred_prev_yaw_error)
                            self.alpha_pred[i-1] += self.alpha_feedback
                            self.pred_prev_yaw_error = yaw_error
                        else:
                            self.alpha_pred[i-1] += self.alpha_feedback
                    velocity_error_pred = TURTLE_VELOCITY - self.velocity_pred[i-1]
                    # if velocity_error_pred >= 0.5*velocity_error:
                    #     self.jerk_pred[i-1] = (pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    # else:
                    #     self.jerk_pred[i-1] = -(pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    #print(self.alpha_pred[i-1])
                    velocity_error = TURTLE_VELOCITY - self.velocity_pred[i-1]
                    self.jerk_pred[i-1] = K_vel*velocity_error_pred + D_vel*(velocity_error-self.pred_prev_vel_error)
                    self.pred_prev_vel_error = velocity_error_pred
                    self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                    self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                    omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                    self.yaw_pred[i] = self.yaw_pred[i-1] + omega_pred[i]*dt
                    self.x_pred[i] = self.x_pred[i-1] + self.velocity*math.cos(self.yaw_pred[i])*dt
                    self.y_pred[i] = self.y_pred[i-1] + self.velocity*math.sin(self.yaw_pred[i])*dt
                    self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                    self.get_future_semantic_position()
                    self.n_step = i
                    self.check_constraints()
                    #print(self.no_enter_constraints_satisfied)
                    if not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
                        #plt.plot(self.x_pred[0:i], self.y_pred[0:i], 'r--')
                        #plt.waitforbuttonpress()
                        self.point_not_satisfied = i
                        break
                    

            if self.action == 'turn right':
                if not self.set_relative_yaw:
                    self.yaw_before_intersect = self.yaw
                    self.set_relative_yaw = True

                for i in range(1,H):
                    if self.pred_semantic_position == 'intersection':
                        if self.yaw_pred[i-1]-self.yaw_before_intersect >= -0.25*math.pi:
                            self.alpha_pred[i-1] = -((self.velocity**2)*math.pi)/(2*(R_right**2))
                        elif self.yaw_pred[i-1]-self.yaw_before_intersect < -0.25*math.pi and self.yaw_pred[i-1] \
                        - self.yaw_before_intersect > -0.5*math.pi:
                            self.alpha_pred[i-1] = ((self.velocity**2)*math.pi)/(2*(R_right**2))
                        elif self.yaw_pred[i-1]-self.yaw_before_intersect <= -0.5*math.pi:
                            self.alpha_pred[i-1] = 0
                        self.alpha_pred[i-1] += self.alpha_feedback

                    elif self.pred_semantic_position == 'lane': 
                        if self.alpha_feedback == 0:  
                            yaw_error = math.atan2(self.pred_direction[1], self.pred_direction[0]) - self.yaw_pred[i-1]
                            #print(yaw_error)
                            if yaw_error > math.pi:
                                yaw_error -= 2*math.pi
                            if yaw_error < -math.pi:
                                yaw_error += 2*math.pi                       
                            self.alpha_pred[i-1] = K_angle*yaw_error + D_angle*(yaw_error-self.pred_prev_yaw_error)
                            self.pred_prev_yaw_error = yaw_error
                            #self.alpha_pred[i-1] = 0
                        else:
                            self.alpha_pred[i-1] += self.alpha_feedback
                    velocity_error_pred = TURTLE_VELOCITY - self.velocity_pred[i-1]
                    # if velocity_error_pred >= 0.5*velocity_error:
                    #     self.jerk_pred[i-1] = (pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    # else:
                    #     self.jerk_pred[i-1] = -(pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    self.jerk_pred[i-1] = K_vel*velocity_error_pred + D_vel*(velocity_error-self.pred_prev_vel_error)
                    self.pred_prev_vel_error = velocity_error_pred
                    self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                    self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                    omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                    #print(omega_pred[i])
                    self.yaw_pred[i] = self.yaw_pred[i-1] + omega_pred[i]*dt
                    #print(self.yaw_pred[i])
                    self.x_pred[i] = self.x_pred[i-1] + self.velocity_pred[i]*math.cos(self.yaw_pred[i])*dt
                    self.y_pred[i] = self.y_pred[i-1] + self.velocity_pred[i]*math.sin(self.yaw_pred[i])*dt
                    self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                    self.get_future_semantic_position()
                

            if self.action == 'slow down and stop':
                for i in range(1,H):
                    if not self.pred_in_stop_area:
                        if self.stop < stop_dist/2:
                            self.jerk_pred[i-1] = -(pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                        else:
                            self.jerk_pred[i-1] = (pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    elif self.pred_in_stop_area:
                        velocity_error = 0 - self.velocity_pred[i]
                        self.jerk_pred[i-1] = 0.1*velocity_error
                    #print(self.jerk_pred[i-1])
                    self.alpha_pred[i-1] = 0
                    omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                    self.yaw_pred[i] = self.yaw_pred[i-1] + omega_pred[i]*dt
                    self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                    self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                    self.x_pred[i] = self.x_pred[i-1] + self.velocity_pred[i]*math.cos(self.yaw_pred[i])*dt
                    self.y_pred[i] = self.y_pred[i-1] + self.velocity_pred[i]*math.sin(self.yaw_pred[i])*dt
                    self.stop += self.velocity_pred[i]*math.sin(self.yaw_pred[i])*dt
                    self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                    self.get_future_semantic_position()                  

            if not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
                plt.plot(self.x_pred[0:self.point_not_satisfied], self.y_pred[0:self.point_not_satisfied], 'r--')
                #print(self.alpha_feedback)
                #plt.waitforbuttonpress()
            #self.check_constraints()
            #print(self.alpha_feedback)       
        plt.plot(self.x_pred, self.y_pred, 'b--')

    def predict2(self):
        self.x_pred = np.zeros(H)
        self.y_pred = np.zeros(H)
        self.yaw_pred = np.zeros(H)
        self.acc_pred = np.zeros(H)
        self.velocity_pred = np.zeros(H)
        self.omega_pred = np.zeros(H)
        self.x_pred[0] = self.pos[0]
        self.y_pred[0] = self.pos[1]
        self.yaw_pred[0] = self.yaw
        self.acc_pred[0] = self.acc
        self.velocity_pred[0] = self.velocity
        self.omega_pred[0] = self.omega
        self.alpha_pred = np.zeros(H-1)
        self.jerk_pred = np.zeros(H-1)

        self.alpha_feedback = 0
        self.stop = 0
        stop_dist = d/2
        
        self.pred_semantic_position = self.semantic_position
        self.pred_in_stop_area = self.in_stop_area
        self.future_robot_area = self.robot_area
        self.pred_direction = self.current_direction
        self.pred_prev_yaw_error = 0
        velocity_error = TURTLE_VELOCITY - self.velocity

        self.no_enter_constraints_satisfied = True
        self.avoid_constraints_satisfied = True

        self.set_turn_timer = False
        #self.turning = False

       
        #while not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
            #self.point_not_satisfied = 0
        #print(self.action)
        if self.action == 'drive straight':          # deze hoeft eigenlijk niet, dit moet ie gwn altijd doen als ie in de lane is
            for i in range(1,H):
                self.x_pred[i] = self.x_pred[i-1] + self.velocity*math.cos(self.yaw)*dt
                self.y_pred[i] = self.y_pred[i-1] + self.velocity*math.sin(self.yaw)*dt
            self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
            self.get_future_semantic_position()
            #self.plot_constraints()
            #plt.plot(self.x_pred, self.y_pred, 'r--')
            #plt.waitforbuttonpress()
            

        if self.action == 'turn left':
            yaw_des = 0.5*math.pi
            #T12 = ((yaw_des)-alpha_max*(T1**2))/omega_max
            #T2 = T1 + T12
            #T3 = T2 + T1
            T_omega = yaw_des/omega_max
            turn_dist = self.velocity/omega_max
            turn_pos = ((self.world.map_dict[self.lane_id_goal]['geometry'][4]+self.world.map_dict[self.lane_id_goal]['geometry'][6])/2)-turn_dist
            for i in range(1,H):
                if self.y_pred[i-1] > turn_pos and not self.turning[i-1] and not self.action_finished[i-1]:
                    self.turning[i-1] = True
                    self.time_turning[i-1] = 0

                if self.time_turning[i-1] >= T_omega:
                    self.action_finished[i-1] = True
                    self.turning[i-1] = False

                if self.turning[i-1]:
                    self.omega_pred[i-1] = omega_max
                    self.time_turning[i-1] += dt

                elif not self.turning[i-1]:
                    self.omega_pred[i-1] = 0
            
                self.jerk_pred[i-1] = 0
                self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                #omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                #omega_pred[i] = omega_max
                #print(f'Omega: {omega_pred[i]}')
                self.yaw_pred[i] = self.yaw_pred[i-1] + self.omega_pred[i-1]*dt
                self.x_pred[i] = self.x_pred[i-1] + self.velocity*math.cos(self.yaw_pred[i])*dt
                self.y_pred[i] = self.y_pred[i-1] + self.velocity*math.sin(self.yaw_pred[i])*dt
                self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                self.get_future_semantic_position()
                self.n_step = i


                self.check_constraints()
                    # #print(self.no_enter_constraints_satisfied)
                if not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
                    plt.plot(self.x_pred[0:i], self.y_pred[0:i], 'r--')
                    plt.waitforbuttonpress()
                    #     self.point_not_satisfied = i
                    #     break
                    

        if self.action == 'turn right':
            yaw_des = 0.5*math.pi
            #T12 = ((yaw_des)-alpha_max*(T1**2))/omega_max
            #T2 = T1 + T12
            #T3 = T2 + T1
            T_omega = yaw_des/omega_max
            turn_dist = self.velocity/omega_max
            turn_pos = ((self.world.map_dict[self.lane_id_goal]['geometry'][4]+self.world.map_dict[self.lane_id_goal]['geometry'][6])/2)-turn_dist
            for i in range(1,H):
                if self.y_pred[i-1] > turn_pos and not self.turning[i-1] and not self.action_finished[i-1]:
                    self.turning[i-1] = True
                    self.time_turning[i-1] = 0

                if self.time_turning[i-1] >= T_omega:
                    self.action_finished[i-1] = True
                    self.turning[i-1] = False

                if self.turning[i-1]:
                    self.omega_pred[i-1] = -omega_max
                    self.time_turning[i-1] += dt

                elif not self.turning[i-1]:
                    self.omega_pred[i-1] = 0
            
                self.jerk_pred[i-1] = 0
                self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                #omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                #omega_pred[i] = omega_max
                #print(f'Omega: {omega_pred[i]}')
                self.yaw_pred[i] = self.yaw_pred[i-1] + self.omega_pred[i-1]*dt
                self.x_pred[i] = self.x_pred[i-1] + self.velocity*math.cos(self.yaw_pred[i])*dt
                self.y_pred[i] = self.y_pred[i-1] + self.velocity*math.sin(self.yaw_pred[i])*dt
                self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                self.get_future_semantic_position()
                self.n_step = i


                self.check_constraints()
                    # #print(self.no_enter_constraints_satisfied)
                if not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
                    plt.plot(self.x_pred[0:i], self.y_pred[0:i], 'r--')
                    plt.waitforbuttonpress()
                    #     self.point_not_satisfied = i
                    #     break
                

            if self.action == 'slow down and stop':
                for i in range(1,H):
                    if not self.pred_in_stop_area:
                        if self.stop < stop_dist/2:
                            self.jerk_pred[i-1] = -(pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                        else:
                            self.jerk_pred[i-1] = (pow(self.velocity_pred[i-1],3)/pow(stop_dist,2))
                    elif self.pred_in_stop_area:
                        velocity_error = 0 - self.velocity_pred[i]
                        self.jerk_pred[i-1] = 0.1*velocity_error
                    #print(self.jerk_pred[i-1])
                    self.alpha_pred[i-1] = 0
                    omega_pred[i] = omega_pred[i-1] + self.alpha_pred[i-1]*dt
                    self.yaw_pred[i] = self.yaw_pred[i-1] + omega_pred[i]*dt
                    self.acc_pred[i] = self.acc_pred[i-1] + self.jerk_pred[i-1]*dt
                    self.velocity_pred[i] = self.velocity_pred[i-1] + self.acc_pred[i]*dt
                    self.x_pred[i] = self.x_pred[i-1] + self.velocity_pred[i]*math.cos(self.yaw_pred[i])*dt
                    self.y_pred[i] = self.y_pred[i-1] + self.velocity_pred[i]*math.sin(self.yaw_pred[i])*dt
                    self.stop += self.velocity_pred[i]*math.sin(self.yaw_pred[i])*dt
                    self.get_future_robot_area(self.x_pred[i], self.y_pred[i], self.yaw_pred[i])
                    self.get_future_semantic_position()                  

            if not self.no_enter_constraints_satisfied or not self.avoid_constraints_satisfied:
                plt.plot(self.x_pred[0:self.point_not_satisfied], self.y_pred[0:self.point_not_satisfied], 'r--')
                #print(self.alpha_feedback)
                #plt.waitforbuttonpress()
            #self.check_constraints()
            #print(self.alpha_feedback)       
        plt.plot(self.x_pred, self.y_pred, 'b--')

        
    def check_constraints(self):
        self.no_enter_constraints_satisfied = True
        self.avoid_constraints_satisfied = True
        for constraint in self.no_enter_constraints:
            if self.future_robot_area.intersects(constraint):
                print("No enter constraint is violated")
                self.no_enter_constraints_satisfied = False
                self.alpha_feedback += K_alpha
        for constraint in self.avoid_constraints:
            if self.future_robot_area.intersects(constraint) and self.n_step > 0.1*H:
                print("Avoid constraint is violated")
                self.avoid_constraints_satisfied = False
                self.alpha_feedback -= K_alpha

    def control(self):
        yaw_prev = self.yaw
        omega_prev = self.omega
        vel_prev = self.velocity
        acc_prev = self.acc
        x_prev = self.pos[0]
        y_prev = self.pos[1]
        self.jerk = self.jerk_pred[0]
        self.acc = acc_prev + self.jerk*dt
        self.velocity = vel_prev + self.acc*dt
        #self.alpha = self.alpha_pred[0]
        #self.omega = omega_prev + self.alpha*dt
        self.omega = self.omega_pred[0]
        self.yaw = yaw_prev + self.omega*dt

        self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
        self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
        pass

    def get_robot_area(self): 
        outline = np.array([[-self.length/2, self.length/2, self.length/2, -self.length/2, -self.length/2],
                        [-self.width/2, -self.width/2, self.width/2, self.width/2, -self.width/2]])
        Rot1 = np.array([[math.cos(self.yaw), math.sin(self.yaw)],
                        [-math.sin(self.yaw), math.cos(self.yaw)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += self.pos[0]
        outline[1, :] += self.pos[1]
        self.robot_area = convert_nparray_to_polygon(np.hstack((outline[0, :][:-1], outline[1, :][:-1])))

    def get_future_robot_area(self, x_pred, y_pred, yaw_pred): 
        outline = np.array([[-self.length/2, self.length/2, self.length/2, -self.length/2, -self.length/2],
                        [-self.width/2, -self.width/2, self.width/2, self.width/2, -self.width/2]])
        Rot1 = np.array([[math.cos(yaw_pred), math.sin(yaw_pred)],
                        [-math.sin(yaw_pred), math.cos(yaw_pred)]])
        outline = (outline.T.dot(Rot1)).T
        outline[0, :] += x_pred
        outline[1, :] += y_pred
        self.future_robot_area = convert_nparray_to_polygon(np.hstack((outline[0, :][:-1], outline[1, :][:-1])))

    def get_semantic_position(self):
        for key, value in self.world.map_dict.items():
            if self.robot_area.intersects(self.world.map_dict[key]['poly']):
                if self.world.map_dict[key]['type'] == 'lane' and self.robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.5*self.rob_area:
                    self.current_direction = self.world.map_dict[key]['direction']
                    self.lane_id = key
                    self.semantic_position = 'lane'
                elif self.world.map_dict[key]['type'] == 'on intersection' and self.robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.5*self.rob_area:
                    self.semantic_position = 'intersection'
                self.current_areas[self.world.map_dict[key]['type']] = self.robot_area.intersection(self.world.map_dict[key]['poly']).area
                if self.semantic_position == 'lane' and self.world.map_dict[key]['type'] == 'stop_area' and self.robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.9*self.rob_area:
                    self.in_stop_area = True

    def get_future_semantic_position(self):
        for key, value in self.world.map_dict.items():
            if self.future_robot_area.intersects(self.world.map_dict[key]['poly']):
                if self.world.map_dict[key]['type'] == 'lane' and self.future_robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.5*self.rob_area:
                    self.pred_direction = self.world.map_dict[key]['direction']
                    self.pred_lane_id = key
                    self.pred_semantic_position = 'lane'
                elif self.world.map_dict[key]['type'] == 'on intersection' and self.future_robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.5*self.rob_area:
                    self.pred_semantic_position = 'intersection'
                self.pred_areas[self.world.map_dict[key]['type']] = self.future_robot_area.intersection(self.world.map_dict[key]['poly']).area
                if self.pred_semantic_position == 'lane' and self.world.map_dict[key]['type'] == 'stop_area' and self.future_robot_area.intersection(self.world.map_dict[key]['poly']).area > 0.9*self.rob_area:
                    self.pred_in_stop_area = True
    def plot_robot(self):
        self.get_robot_area()
        plt.fill(*self.robot_area.exterior.xy, color=self.color)

    def plot_goal(self):
        plt.plot(*self.goal_pos,'kx')

    def check_sit(self):
        other_turtles = []
        if self.world.number_of_turtles > 1:
            other_turtles = list(filter(lambda i: i.name != self.name, self.world.turtles))
        if self.current_areas['lane'] > 0.5*self.rob_area:
            self.situation = 'Driving in lane'
            if self.current_areas['at intersection'] > 0.99*self.rob_area:
                if self.world.map_dict[self.lane_id]['color'] == 'lightblue':
                    self.situation = 'Driving in lane, towards intersection'
                    if len(other_turtles) > 0:
                        for turtle in other_turtles:
                            if turtle.current_areas['lane'] > 0.5*turtle.rob_area:
                                if turtle.current_areas['at intersection'] > 0.99*turtle.rob_area:
                                    if self.world.map_dict[turtle.lane_id]['color'] == 'lightblue':
                                        self.check_priority(self.world, other_turtles)
                                        if self.give_priority:
                                            self.situation = 'Give priority'
                                            #if self.current_areas['stop_area'] > 0.99*self.rob_area:
                                            #    self.situation = 'Give priority, stop'
                            if turtle.current_areas['on intersection'] > 0.5*turtle.rob_area:
                                self.situation = 'Other turtle on intersection'
                elif self.world.map_dict[self.lane_id]['color'] == 'sandybrown':
                    self.situation = 'Driving in lane, from intersection'
        if self.current_areas['on intersection'] > 0.5*self.rob_area:
            self.situation = "Driving on intersection"
            if len(other_turtles) > 0:
                for turtle in other_turtles:
                    if turtle.current_areas['on intersection'] > 0.5*turtle.rob_area:
                        self.situation = "Other turtle on intersection"


    def plot_situation(self):
        plt.title("Situation: " + self.situation)

    def return_action(self):
        if self.situation == 'Give priority' or self.situation == 'Give priority, stop':
            self.action = 'slow down and stop'
            return
        # if self.situation == 'Give priority, stop':
        #     self.action = 'stop'
        #     return
        #if self.situation == 'Driving in lane':
         #   self.action = 'drive straight'
        #elif self.situation == 'Driving on intersection':
        current_lane = self.world.map_dict[self.lane_id]['location']
        if current_lane == 'down':
            if self.goal == 'up':
                self.action = 'drive straight'
            elif self.goal == 'right':
                self.action = 'turn right'
            elif self.goal == 'left':
                self.action = 'turn left'
        elif current_lane == 'up':
            if self.goal == 'down':
                self.action = 'drive straight'
            elif self.goal == 'right':
                self.action = 'turn left'
            elif self.goal == 'left':
                self.action = 'turn right'
        elif current_lane == 'right':
            if self.goal == 'left':
                self.action = 'drive straight'
            elif self.goal == 'up':
                self.action = 'turn right'
            elif self.goal == 'down':
                self.action = 'turn left'
        elif current_lane == 'left':
            if self.goal == 'right':
                self.action = 'drive straight'
            elif self.goal == 'up':
                self.action = 'turn left'
            elif self.goal == 'down':
                self.action = 'turn right'

                    


    def check_priority(self, world, other_turtles):
        for turtle in other_turtles:
            if world.map_dict[turtle.lane_id]['location'] == 'right' and world.map_dict[self.lane_id]['location'] == 'down':
                self.give_priority = True
            if world.map_dict[turtle.lane_id]['location'] == 'down' and world.map_dict[self.lane_id]['location'] == 'left':
                self.give_priority = True
            if world.map_dict[turtle.lane_id]['location'] == 'left' and world.map_dict[self.lane_id]['location'] == 'up':
                self.give_priority = True
            if world.map_dict[turtle.lane_id]['location'] == 'up' and world.map_dict[self.lane_id]['location'] == 'right':
                self.give_priority = True



class GlobalPlanner():
    def __init__(self, robot):
        #self.stepsize = 0.1
        self.before_intersect = robot.before_intersect
        self.after_intersect = robot.after_intersect
        self.robot_pos = robot.pos
        self.robot_yaw = robot.yaw
        self.goal_pos = robot.goal_pos
        self.goal_yaw = robot.goal_yaw
        self.start = robot.start
        self.goal = robot.goal
        
    def straight_line(self, point1, point2):
        x = np.linspace(point1[0], point2[0])
        y = np.linspace(point1[1], point2[1])
        return x,y

    def curve_line(self, point1, point2, yaw):
        x = y = dyaw = np.zeros(len(yaw))

        if (self.start == 'down' and self.goal == 'left') or (self.start == 'up' and self.goal == 'right') \
        or (self.start == 'left' and self.goal == 'up') or (self.start == 'right' and self.goal == 'down'):
            dyaw = yaw - self.robot_yaw
            curve_radius = 15
        elif (self.start == 'down' and self.goal == 'right') or (self.start == 'up' and self.goal == 'left') \
        or (self.start == 'left' and self.goal == 'down') or (self.start == 'right' and self.goal == 'up'):
            dyaw = self.robot_yaw - yaw
            curve_radius = 5
                
        if self.start == 'down' and self.goal == 'left':
            x = point1[0] - curve_radius*(1-np.cos(dyaw))
            y = point1[1] + curve_radius*np.sin(dyaw)
        if self.start == 'down' and self.goal == 'right':
            x = point1[0] + curve_radius*(1-np.cos(dyaw))
            y = point1[1] + curve_radius*np.sin(dyaw)
        if self.start == 'up' and self.goal == 'right':
            x = point1[0] + curve_radius*(1-np.cos(dyaw))
            y = point1[1] - curve_radius*np.sin(dyaw)
        if self.start == 'up' and self.goal == 'left':
            x = point1[0] - curve_radius*(1-np.cos(dyaw))
            y = point1[1] - curve_radius*np.sin(dyaw)
        if self.start == 'right' and self.goal == 'down':
            x = point1[0] - curve_radius*np.sin(dyaw)
            y = point1[1] - curve_radius*(1-np.cos(dyaw))
        if self.start == 'right' and self.goal == 'up':
            x = point1[0] - curve_radius*np.sin(dyaw)
            y = point1[1] + curve_radius*(1-np.cos(dyaw))
        if self.start == 'left' and self.goal == 'up':
            x = point1[0] + curve_radius*np.sin(dyaw)
            y = point1[1] + curve_radius*(1-np.cos(dyaw))
        if self.start == 'left' and self.goal == 'down':
            x = point1[0] + curve_radius*np.sin(dyaw)
            y = point1[1] - curve_radius*(1-np.cos(dyaw))
        return x,y


    def create_trajectory(self):
        x = y = yaw = np.zeros(150)
        if (self.start == 'up' and self.goal == 'down') or (self.start == 'down' and self.goal == 'up') \
        or (self.start == 'left' and self.start == 'right') or (self.start == 'right' and self.start == 'left'):
            yaw = np.linspace(self.robot_yaw, self.goal_yaw, num=150)
            x = np.linspace(self.robot_pos[0], self.goal_pos[0], num=150)
            y = np.linspace(self.robot_pos[1], self.goal_pos[1], num=150)

        else:
            yaw_vertical = np.linspace(self.robot_yaw, self.robot_yaw)
            yaw_turn = np.linspace(self.robot_yaw, self.goal_yaw)
            yaw_horizontal = np.linspace(self.goal_yaw, self.goal_yaw)

            x_vertical, y_vertical = self.straight_line(self.robot_pos, self.before_intersect)
            x_turn, y_turn = self.curve_line(self.before_intersect, self.after_intersect, yaw_turn)
            x_horizontal, y_horizontal = self.straight_line(self.after_intersect, self.goal_pos)

            x = [*x_vertical, *x_turn, *x_horizontal]
            y = [*y_vertical, *y_turn, *y_horizontal]
            yaw = [*yaw_vertical, *yaw_turn, *yaw_horizontal]
        self.trajectory = [x,y,yaw]

    def get_trajectory(self):
        return self.trajectory

    def plot_trajectory(self):
        x, y, yaw = self.trajectory
        plt.plot(x, y, 'b--')