from queries import *
from basic_functions import *
import matplotlib.pyplot as plt
import numpy as np
import geopandas
import math
from shapely.geometry import Polygon, LineString
from rdflib import URIRef
from global_variables import EX



class Perception():
    def __init__(self):
        self.spotted_sign = False
        self.pos = 0

    def odometry(self, world, robot):
        pass

    def perceive_query(self, world, inputs):       # de skills moeten eigenlijk vragen wat nodig is
        #world.clear_areas()
        road = world.start
        if len(inputs)==2:
            self.pos = road
        elif len(inputs) > 2:
            self.pos = query_part_of(road)


        
        ## dit is de perceive road skill
        
  
        #robot_pos_polygon = query_if_polygon(robot_pos)
        #parts = has_a(self.pos)
        parts = has_first_layer_geometries(self.pos)
        #print(parts)
        self.perceivable_road_parts = []
        self.not_perceivable_road_parts = []
        n = 0

        ## seperating what can be perceived, for intersection moet ik eigenlijk nog n laag lager gaan 
        for part in parts:
            perceivable = query_if_perceivable(part)
            
            if perceivable:
                self.perceivable_road_parts.append(part)

            if not perceivable:
                self.not_perceivable_road_parts.append(part)

        print(self.perceivable_road_parts)
        #print(self.not_perceivable_road_parts)

    def perceive_geom(self, world, simulator, inputs):
        xaxis = LineString(([-30, 0],[30, 0]))
        robot = simulator.robots[0]        
        #polygon_uri = URIRef("http://example.com/polygon")
        #line_uri = URIRef("http://example.com/line")
        ## road asociation skill, later seperate functions
        # dit moet niet altijd gebeuren, maar gwn een keer, vervolgens alleen nog een vorm van checken (monitor)
        for input in inputs:
            intersect = input.intersection(xaxis)
            #print(intersection)
            if intersect:
                if intersect.coords[0][0] <= 0:
                    left_part = query_left_part(self.perceivable_road_parts)
                    #print(f"input: {input}")
                    world.add_area(robot, input, left_part)
                elif intersect.coords[0][0] > 0:
                    right_part = query_right_part(self.perceivable_road_parts)
                    world.add_area(robot, input, right_part)

        ## moet nog zorgen dat het geen error geeft als het een side niet meer ziet, miss data van de vorige run meenemen
        
        sub_part_areas = []
        for part in self.not_perceivable_road_parts:
            sub_parts = has_a(part)

            #print(part)
            for sub_part in sub_parts:
                for perceivable_part in self.perceivable_road_parts:
                    is_equal = query_if_equal(sub_part, perceivable_part)
                    if is_equal:
                        #print('hello')
                        sub_part_areas.append(world.get_area(perceivable_part))

        lane_polygon = Polygon([sub_part_areas[0].coords[0], sub_part_areas[0].coords[1], sub_part_areas[1].coords[1], sub_part_areas[1].coords[0]])
        world.add_area(robot, lane_polygon, self.not_perceivable_road_parts[0])

        if False:
            if len(inputs) > 2:
                intersection = query_part_of(road)
                intersection_parts = has_a(intersection)
                #for part in intersection_parts:
                    #print(part)
                for i in range(len(inputs)):
                    for j in range(len(inputs)):
                        line_intersect = inputs[i].intersection(inputs[j])

                        if line_intersect.geom_type=='Point':
                            print(f'corner found at {line_intersect}')
                            print(inputs[i])

        #world.plot_areas()

        # link sides met linestrings, gebruik relaties tussen sides en lane om de lane te plotten, het is in dit geval simpel maar moet altijd werken (intersection)

        #print(inputs)
        #print(road_parts)


        # robot = simulator.robots[0]
        # input_strings = list(filter(lambda x: isinstance(x, str), args))
        # input_features = list(filter(lambda x: isinstance(x, Polygon), args))
        # sign = self.check_labels_in_graph(input_strings)
        # if not self.spotted_sign:
        #     self.set_map(world, input_features)
        # elif self.spotted_sign:                         # we will asume it does spot a sign (road or intersection)
        #     world.update_sit(self.spotted_sign)
        #     areas, uris = self.query_topology(robot, sign)
        #     self.set_map(world, areas, uris)

    def check_labels_in_graph(self, input_strings):
        uris = []
        #labels = []
        for string in input_strings:
            uri = uri_from_label(string)
            if uri != 0:
                uris.append(uri)
                #labels.append(string)
        if not uris:
            print('NO PRIOR KNOWLEDGE AVAILABLE')
            self.spotted_sign = False
        elif uris:
            if len(uris) > 1:
                print("SEEING MORE THAN ONE SIGN!, ASSUMING FIRST SIGN")    # zou later aan de hand van features nog kunnen kijken welke het meest waarschijnlijk is
            self.spotted_sign = True
            return uris[0]

    def set_map(self, world, areas, uris):
        for area, uri in zip(areas, uris):
            #print(area)
            #print(uri)
            world.add_area(area, uri)
        world.plot_areas()

    def query_topology(self, robot, sign):
        ## hier kan nog veel aan veranderen, geef voor nu even gwn de areas die eruit zouden moeten komen
        areas=[]
        uris = []
        uri_sign = str(*sign)
        uri_road = 'http://example.com/road'
        uri_intersection = 'http://example.com/intersection'
        if uri_sign == uri_road:
            side1 = abs_to_rel(robot, np.array([60.0, 40.0]), 80, 1)
            lane = abs_to_rel(robot, np.array([50.0, 40.0]), 80, 20)
            side2 = abs_to_rel(robot, np.array([40.0, 40.0]), 80, 1)
            areas = [lane, side1, side2]
            lane_uri = URIRef(uri_road + "/lane")
            side1_uri = URIRef(uri_road + "/side1")
            side2_uri = URIRef(uri_road + "/side2")
            uris = [lane_uri, side1_uri, side2_uri]
        #print(areas)
        elif uri_sign == uri_intersection:
            side1 = abs_to_rel(robot, np.array([60.0, 20.0]), 40, 1)
            side2 = abs_to_rel(robot, np.array([40.0, 20.0]), 40, 1)
            side3 = abs_to_rel(robot, np.array([80.0, 40.0]), 1, 40)
            side4 = abs_to_rel(robot, np.array([80.0, 60.0]), 1, 40)
            side5 = abs_to_rel(robot, np.array([60.0, 80.0]), 40, 1)
            side6 = abs_to_rel(robot, np.array([40.0, 80.0]), 40, 1)
            side7 = abs_to_rel(robot, np.array([20.0, 60.0]), 1, 40)
            side8 = abs_to_rel(robot, np.array([20.0, 40.0]), 1, 40)
            lane1 = abs_to_rel(robot, np.array([50.0, 20.0]), 40, 20)
            lane2 = abs_to_rel(robot, np.array([80.0, 50.0]), 20, 40)
            lane3 = abs_to_rel(robot, np.array([50.0, 80.0]), 40, 20)
            lane4 = abs_to_rel(robot, np.array([20.0, 50.0]), 20, 40)
            middle = abs_to_rel(robot, np.array([50.0, 50.0]), 20, 20)

            areas = [middle, lane1, lane2, lane3, lane4, side1, side2, side3, side4, side5, side6, side7, side8]
            middle_uri = URIRef(uri_intersection + "/middle")
            lane1_uri = URIRef(uri_intersection + "/road1/lane")
            lane2_uri = URIRef(uri_intersection + "/road2/lane")
            lane3_uri = URIRef(uri_intersection + "/road3/lane")
            lane4_uri = URIRef(uri_intersection + "/road4/lane")
            side1_uri = URIRef(uri_intersection + "/road1/side1")
            side2_uri = URIRef(uri_intersection + "/road1/side2")
            side3_uri = URIRef(uri_intersection + "/road2/side1")
            side4_uri = URIRef(uri_intersection + "/road2/side2")
            side5_uri = URIRef(uri_intersection + "/road3/side1")
            side6_uri = URIRef(uri_intersection + "/road3/side2")
            side7_uri = URIRef(uri_intersection + "/road4/side1")
            side8_uri = URIRef(uri_intersection + "/road4/side2")

            uris = [middle_uri, lane1_uri, lane2_uri, lane3_uri, lane4_uri, side1_uri, side2_uri, side3_uri, side4_uri, side5_uri, side6_uri, side7_uri, side8_uri]
        return areas, uris


    def recognize_sit(self, sit, side):
        self.traffic_sign = sit
        self.side = side

    def query_relations(self, world):
        uri = uri_from_label(world.g, self.traffic_sign)
        #print(uri)
        self.parts = has_a(world.g, uri)
        self.sides = has_sides(world.g, uri)     # hier miss nog met n attribute "can be perceived" oid aangeven wrm de sides gequeried worden
        world.add_area(self.sides[0], self.side)
        geopandas.GeoDataFrame({})

        #print(self.parts)

        # conceptuele side die in parts zit vergelijken met de side die gezien is
        # eerst een side los queryen en dan linken met de polygon
        # eerste die gezien is is side1, ze hebben verder geen verschil dus dit lijkt me prima methode

        #query distance between sides   dit moet dus nog in de geografischae db gebeuren
        #self.distance = distance_sides_road(world.g)

        
    def configure_map(self, world, robot):
        side_abs_pos = world.areas[self.sides[0]]['abs_pos']

        # dit moet miss toch net iets anders. de relatie is gwn n bepaalde relatieve afstand,
        # dus die kun je gwn als relatie erin zetten, alleen of het min of plus die afstand hier is ligt aan welke kan je hebt
        # je moet het eigenlijk kunnen afleiden aan welke kant van de side je ziet of waar de robot nu zelf is
        lane = abs_to_rel(robot, np.array([side_abs_pos[0]-0.5*float(*self.distance), side_abs_pos[1]]), 80, 20)
        other_side = abs_to_rel(robot, np.array([side_abs_pos[0]-float(*self.distance), side_abs_pos[1]]), 80, 1)


        world.add_area(self.sides[1], other_side)
        world.add_area(self.parts[0], lane)
        world.plot_areas()

    def get_current_pos(self, world, robot):
        for area in world.areas.values():
            print(robot.intersection(area['polygon']))

 