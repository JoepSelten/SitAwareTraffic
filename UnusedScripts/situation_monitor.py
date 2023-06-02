from shapely.geometry import Polygon, Point, LineString, CAP_STYLE, box
import numpy as np
import math
import matplotlib.pyplot as plt

class SituationMonitor():
    def __init__(self):
        self.situation = 'Idle'
    
    def sit_check(self, turtle1, world):
        other_turtles = []
        if world.number_of_turtles > 1:
            other_turtles = list(filter(lambda i: i.name != turtle1.name, world.turtles))
        if turtle1.current_areas['lane'] > 0.5*turtle1.rob_area:
            self.situation = 'Driving in lane'
            if turtle1.current_areas['at intersection'] > 0.99*turtle1.rob_area:
                if world.map_dict[turtle1.lane_id]['color'] == 'lightblue':
                    self.situation = 'Driving in lane, towards intersection'
                    if len(other_turtles) > 0:
                        for turtle in other_turtles:
                            if turtle.current_areas['lane'] > 0.5*turtle.rob_area:
                                #if turtle.lane_id == turtle1.lane_id:
                                   #self.situation = 'Driving on the same lane'
                                if turtle.current_areas['at intersection'] > 0.99*turtle.rob_area:
                                    if world.map_dict[turtle.lane_id]['color'] == 'lightblue':
                                        self.situation = 'Other turtle before intersection'
                            if turtle.current_areas['on intersection'] > 0.5*turtle.rob_area:
                                self.situation = 'Other turtle on intersection'
                elif world.map_dict[turtle1.lane_id]['color'] == 'sandybrown':
                    self.situation = 'Driving in lane, from intersection'
        if turtle1.current_areas['on intersection'] > 0.5*turtle1.rob_area:
            self.situation = "Driving on intersection"
            if len(other_turtles) > 0:
                for turtle in other_turtles:
                    if turtle.current_areas['on intersection'] > 0.5*turtle.rob_area:
                        self.situation = "Other turtle on intersection"
            #     self.situation = 'Driving at intersection'
        return self.situation
                
    
    def plot_situation(self):
        plt.title("Situation: " + self.situation)
        #plt.text(0.05, 0.95, "hallo")
        pass

    def region_check(self):
        pass
