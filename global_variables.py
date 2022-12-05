from rdflib import Graph, Namespace
from rdflib.namespace import GEO
import json

## set configs
with open('conf.json') as f:
    config = json.load(f)
    f.close

TURTLE_LENGTH = config['turtle_length']
TURTLE_WIDTH = config['turtle_width']
TURTLE_VELOCITY = config['turtle_velocity']
ROAD_LENGTH = config['road_length']
ROAD_WIDTH = config['road_width']
dt = config['dt']
prediction_horizon = config['prediction_horizon']
H = round(prediction_horizon/dt)
b = config['boundary_thickness']
l = config['road_length']
w = config['road_width']
c = config['stop_area_size']


## load property graph , later miss met externe server doen oid
g = Graph()
g.parse("kg3.json", format="json-ld")
EX = Namespace("http://example.com/") 
GEO = Namespace("http://www.opengis.net/ont/geosparql#")
g.bind('ex', EX)
g.bind('geo', GEO)
