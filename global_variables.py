from rdflib import Graph, Namespace
from rdflib.namespace import GEO
import json

## set configs
with open('conf.json') as f:
    config = json.load(f)
    f.close

AV_LENGTH = config['av_length']
AV_WIDTH = config['av_width']
AV_VELOCITY = config['av_velocity']
AV_OMEGA = config['av_omega']
ROAD_LENGTH = config['road_length']
ROAD_WIDTH = config['road_width']
PERCEPTION_RADIUS = config['perception_radius']
dt = config['dt']
prediction_horizon = config['prediction_horizon']
H = round(prediction_horizon/dt)
b = config['boundary_thickness']
l = config['road_length']
w = config['road_width']
c = config['stop_area_size']

MAX_TURN_DISTANCE = 5
APPROACH_DISTANCE = 20

## load property graph , later miss met externe server doen oid
#g = Graph(store="Oxigraph")
g = Graph()
g.parse("kg4.json", format="json-ld")
EX = Namespace("http://example.com/") 
GEO = Namespace("http://www.opengis.net/ont/geosparql#")
g.bind('ex', EX)
g.bind('geo', GEO)
