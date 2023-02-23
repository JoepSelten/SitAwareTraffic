import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt

class Control():
    def __init__(self):
        pass

    def execute_skill(self, world):
        skill = world.skill
        if skill == 'move_in_lane':
            self.move_in_lane(world)
        elif skill == 'turn':
            self.turn(world)
        else:
            self.stop()

    def move_in_lane(self, world, phi_des):
        # this skill requires the orientation of the lane
        prev_pos = world.robot.pos[0]
        yaw_error = world.robot.yaw - phi_des
        self.velocity = world.robot.velocity
        self.omega = -yaw_error
        world.skill_finished = True

    def turn(self, world, phi_des, centerline):
        self.velocity = world.robot.velocity
        reduced_velocity = False
        yaw_error = world.robot.yaw - phi_des
        T_omega = phi_des/world.robot.omega_max
        turn_dist = self.velocity/world.robot.omega_max
    
        if turn_dist > 5:
            turn_dist = 5
            reduced_velocity = True
        ## kan later nog n bepaalde skill hebben die altijd afremt voor n intersection
        print(world.robot.point.distance(centerline))
        turn_pos = 55-turn_dist
        
        if world.robot.pos[1] < turn_pos:
            self.omega = 0
        elif world.robot.pos[1] >= turn_pos and abs(yaw_error) > 0.05:
            self.omega = -1*np.sign(yaw_error)*world.robot.omega_max
            if reduced_velocity:
                self.velocity = world.robot.omega_max*5

        #elif world.robot.pos[1] >= turn_pos and yaw_error > 0.05:
         #   self.omega = -world.robot.omega_max
        else:
            self.omega = 0
            world.skill_finished = True

    def stop(self, world):
        self.velocity = 0
        self.omega = 0
        world.skill_finished = True


    def actuate(self, simulator):
        AV = simulator.robots[0]
        simulator.move_robot(AV, self.velocity, self.omega)
        # prev_yaw = AV.yaw
        # prev_pos = AV.pos

        # self.yaw = prev_yaw + self.omega*dt
        # self.pos[0] = x_prev + self.velocity*math.cos(self.yaw)*dt
        # self.pos[1] = y_prev + self.velocity*math.sin(self.yaw)*dt
        # simulator.robots[0].pos = self.pos




                
                

