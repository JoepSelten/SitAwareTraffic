from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from closure_graph import Semantics
from owlrl import DeductiveClosure
from urllib.parse import urlparse, urljoin


## Namespaces
EX = Namespace("http://example.com/")
#GEO = Namespace("http://www.opengis.net/ont/GEOsparql#")

g = Graph()
#g = Graph(store="Oxigraph")
g.bind('ex', EX)
#g.bind('geo', GEO)

def add_has_a(g, subject, object):
    uri = URIRef(urljoin(subject + '/', object.split('.com/')[-1]))
    g.add((subject, EX.has_a, uri))
    return uri

## labels (like a traffic sign)
g.add((EX.road, RDFS.label, Literal('road')))
g.add((EX.intersection, RDFS.label, Literal('intersection')))

## Mereology
intersection_crossing = add_has_a(g, EX.intersection, EX.crossing)
intersection_road_down = add_has_a(g, EX.intersection, EX.road_down)
intersection_road_right = add_has_a(g, EX.intersection, EX.road_right)
intersection_road_up = add_has_a(g, EX.intersection, EX.road_up)
intersection_road_left = add_has_a(g, EX.intersection, EX.road_left)

g.add((intersection_crossing, RDF.type, EX.crossing))
g.add((intersection_road_down, RDF.type, EX.road))
g.add((intersection_road_right, RDF.type, EX.road))
g.add((intersection_road_up, RDF.type, EX.road))
g.add((intersection_road_left, RDF.type, EX.road))

road_lane1 = add_has_a(g, EX.road, EX.lane1)
road_lane2 = add_has_a(g, EX.road, EX.lane2)

#g.add((road_lane1, RDF.type, EX.lane_right))
#g.add((road_lane2, RDF.type, EX.lane_left))



## Topology
g.add((road_lane1, EX.connects, road_lane2))
g.add((intersection_crossing, EX.connects, intersection_road_down))
g.add((intersection_crossing, EX.connects, intersection_road_right))
g.add((intersection_crossing, EX.connects, intersection_road_up))
g.add((intersection_crossing, EX.connects, intersection_road_left))


## Affordances
g.add((EX.intersection, RDF.type, EX.geometry))
g.add((EX.road, RDF.type, EX.geometry))
g.add((EX.crossing, RDF.type, EX.geometry))
#g.add((EX.lane, RDF.type, EX.geometry))
g.add((EX.lane_right, RDF.type, EX.geometry))
g.add((EX.lane_left, RDF.type, EX.geometry))
#g.add((EX.polygon, RDF.type, EX.geometry))
#g.add((EX.lane_right, RDF.type, EX.geometry))
#g.add((EX.lane_left, RDF.type, EX.geometry))
g.add((EX.lane_right, EX.affordance, EX.drivable))
g.add((EX.lane_left, EX.affordance, EX.drivable))
g.add((EX.crossing, EX.affordance, EX.drivable))
g.add((EX.lane_right, EX.affordance, EX.waiting))

#g.add((EX.obstacle, RDF.type, EX.polygon))
g.add((EX.obstacle, RDF.type, EX.geometry))

DeductiveClosure(Semantics).expand(g)
DeductiveClosure(Semantics).expand(g)
DeductiveClosure(Semantics).expand(g)

#print(g.serialize())
g.serialize(format="json-ld", destination="kg14.jsonld")
g.serialize(destination="kg14.txt")

