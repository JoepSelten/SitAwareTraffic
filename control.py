import math
import numpy as np
from turtle import pos
from basic_functions import trans, coordinate_transform
import matplotlib.pyplot as plt
from global_variables import H, dt, MAX_TURN_DISTANCE

class Control():
    def __init__(self):
        pass

    def execute_behaviour(self, world):
        behaviour = world.behaviour
        if behaviour == 'move_in_lane':
            self.move_in_lane(world)
        elif behaviour == 'turn':
            self.turn(world)
        else:
            self.stop()

    def drive(self, world):
        phi_des = world.plan[world.current_pos]['phi']
        vel_des = world.plan[world.current_pos]['velocity']  
        world.robot.yaw = world.robot.yaw%(2*math.pi)
        phi_des = phi_des%(2*math.pi)
        yaw_error = world.robot.yaw - phi_des
        if abs(yaw_error) > math.pi:
            yaw_error -= np.sign(yaw_error)*2*math.pi
        world.velocity = vel_des
        world.omega = -5*yaw_error
        world.behaviour_finished = True

    def move_in_lane(self, world):
        phi_des = world.plan[str(world.plan_step)]['parameters']['phi']
        vel_des = world.plan[str(world.plan_step)]['parameters']['velocity']
        # this behaviour requires the orientation of the lane      
        world.robot.yaw = world.robot.yaw%(2*math.pi)
        phi_des = phi_des%(2*math.pi)
        yaw_error = world.robot.yaw - phi_des
        if abs(yaw_error) > math.pi:
            yaw_error -= np.sign(yaw_error)*2*math.pi
        world.velocity = vel_des
        world.omega = -yaw_error
        world.behaviour_finished = True

    def turn(self, world):
        phi_des = world.plan[str(world.plan_step)]['parameters']['phi']
        vel_des = world.plan[str(world.plan_step)]['parameters']['velocity']
        extended_centerline = world.plan[str(world.plan_step)]['parameters']['turn_line']

        world.velocity = vel_des
        reduced_velocity = False
        world.robot.yaw = world.robot.yaw%(2*math.pi)
        phi_des = phi_des%(2*math.pi)
        yaw_error = world.robot.yaw - phi_des
        if abs(yaw_error) > math.pi:
            yaw_error -= np.sign(yaw_error)*2*math.pi
        
        turn_dist = world.velocity/world.robot.omega_max
        if turn_dist > MAX_TURN_DISTANCE:
            turn_dist = MAX_TURN_DISTANCE
            reduced_velocity = True
        lane_dist = world.robot.point.distance(extended_centerline)
        
        if lane_dist > turn_dist:
            world.omega = 0
        elif lane_dist <= turn_dist and abs(yaw_error) > 0.05:
            world.omega = -1*np.sign(yaw_error)*world.robot.omega_max
            if reduced_velocity:
                world.velocity = world.robot.omega_max*5
        else:
            world.omega = 0
            world.behaviour_finished = True

    def stop(self, world):
        world.velocity = 0
        world.omega = 0
        world.behaviour_finished = True


    def actuate(self, world, simulator, t):
        if t > world.robot.delay:
            AV = simulator.robots[world.robot.uri]
            simulator.move_robot(AV, world.velocity, world.omega)
 



                
                

