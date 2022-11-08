from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO
from basic_areas import RectangleArea
from queries import uri_from_label, has_a, distance_sides_road, conforms_to_area, has_sides
from basic_functions import abs_to_rel
import matplotlib.pyplot as plt
import numpy as np

class Perception():
    def __init__(self):
        pass

    def recognize_sit(self, sit, side):
        self.traffic_sign = sit
        self.side = side

    def query_relations(self, world):
        uri = uri_from_label(world.g, self.traffic_sign)
        #print(uri)
        self.parts = has_a(world.g, uri)
        self.sides = has_sides(world.g, uri)     # hier miss nog met n attribute "can be perceived" oid aangeven wrm de sides gequeried worden
        world.add_area(self.sides[0], self.side)
        #print(self.parts)

        # conceptuele side die in parts zit vergelijken met de side die gezien is
        # eerst een side los queryen en dan linken met de polygon
        # eerste die gezien is is side1, ze hebben verder geen verschil dus dit lijkt me prima methode

        #query distance between sides
        self.distance = distance_sides_road(world.g)

        
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

 