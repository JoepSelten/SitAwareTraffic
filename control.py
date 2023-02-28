import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt, MAX_TURN_DISTANCE

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
        world.robot.yaw = world.robot.yaw%(2*math.pi)
        phi_des = phi_des%(2*math.pi)
        yaw_error = world.robot.yaw - phi_des
        if abs(yaw_error) > math.pi:
            yaw_error -= np.sign(yaw_error)*2*math.pi
        world.velocity = world.robot.velocity_max
        world.omega = -yaw_error
        world.skill_finished = True

    def turn(self, world, phi_des, extended_centerline):
        world.velocity = world.robot.velocity_max
        reduced_velocity = False
        world.robot.yaw = world.robot.yaw%(2*math.pi)
        phi_des = phi_des%(2*math.pi)
        yaw_error = world.robot.yaw - phi_des
        if abs(yaw_error) > math.pi:
            yaw_error -= np.sign(yaw_error)*2*math.pi

        #print(f'robot_yaw, {world.robot.name}: {world.robot.yaw}')
        #print(f'phi_des, {world.robot.name}: {phi_des}')
        #print(f'yaw_error, {world.robot.name}: {yaw_error}')
        
        turn_dist = world.velocity/world.robot.omega_max
        if turn_dist > MAX_TURN_DISTANCE:
            turn_dist = MAX_TURN_DISTANCE
            reduced_velocity = True
        ## kan later nog n bepaalde skill hebben die altijd afremt voor n intersection
        lane_dist = world.robot.point.distance(extended_centerline)
        
        #print(f'lane_dist, {world.robot.name}: {lane_dist}')
        if lane_dist > turn_dist:
            world.omega = 0
        elif lane_dist <= turn_dist and abs(yaw_error) > 0.05:
            world.omega = -1*np.sign(yaw_error)*world.robot.omega_max
            if reduced_velocity:
                world.velocity = world.robot.omega_max*5
        else:
            world.omega = 0
            world.skill_finished = True

    def stop(self, world):
        world.velocity = 0
        world.omega = 0
        world.skill_finished = True


    def actuate(self, world, simulator):
        AV = simulator.robots[world.robot.name]
        simulator.move_robot(AV, world.velocity, world.omega)
 



                
                

