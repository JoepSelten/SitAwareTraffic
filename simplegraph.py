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
road_lane = add_has_a(g, EX.road, EX.lane)
road_side1 = add_has_a(g, EX.road, EX.side1)
road_side2 = add_has_a(g, EX.road, EX.side2)
g.add((road_lane, RDF.type, EX.lane))
g.add((road_side1, RDF.type, EX.side))
g.add((road_side2, RDF.type, EX.side))

## miss moet ik dit met een closure & query doen
#g.add((EX.intersection, EX.has_a, EX.road1))


intersection_middle = add_has_a(g, EX.intersection, EX.middle)
intersection_road1 = add_has_a(g, EX.intersection, EX.road1)
intersection_road2 = add_has_a(g, EX.intersection, EX.road2)
intersection_road3 = add_has_a(g, EX.intersection, EX.road3)
intersection_road4 = add_has_a(g, EX.intersection, EX.road4)

g.add((intersection_middle, RDF.type, EX.middle))
g.add((intersection_road1, RDF.type, EX.road))
g.add((intersection_road2, RDF.type, EX.road))
g.add((intersection_road3, RDF.type, EX.road))
g.add((intersection_road4, RDF.type, EX.road))

## Topology
g.add((road_lane, EX.connects, road_side1))
g.add((road_lane, EX.connects, road_side2))

g.add((intersection_middle, EX.connects, intersection_road1))
g.add((intersection_middle, EX.connects, intersection_road2))
g.add((intersection_middle, EX.connects, intersection_road3))
g.add((intersection_middle, EX.connects, intersection_road4))

# g.add((EX.lane, EX.conforms_to, EX.polyon))
# g.add((EX.middle, EX.conforms_to, EX.polygon))
# g.add((EX.side, EX.conforms_to, EX.line))

g.add((EX.lane, RDF.type, EX.polyon))
g.add((EX.middle, RDF.type, EX.polygon))
g.add((EX.side, RDF.type, EX.line))

polygon_interior = add_has_a(g, EX.polygon, EX.interior)
polygon_line1 = add_has_a(g, EX.polygon, EX.line1)
polygon_line2 = add_has_a(g, EX.polygon, EX.line2)
polygon_line3 = add_has_a(g, EX.polygon, EX.line3)
polygon_line4 = add_has_a(g, EX.polygon, EX.line4)

g.add((polygon_interior, RDF.type, EX.interior))
g.add((polygon_line1, RDF.type, EX.line))
g.add((polygon_line2, RDF.type, EX.line))
g.add((polygon_line3, RDF.type, EX.line))
g.add((polygon_line4, RDF.type, EX.line))

# g.add((EX.interior, RDF.type, EX.polygon))
# g.add((EX.line1, RDF.type, EX.line))
# g.add((EX.line2, RDF.type, EX.line))
# g.add((EX.line3, RDF.type, EX.line))
# g.add((EX.line4, RDF.type, EX.line))


# ## Affordances
g.add((EX.lane, EX.affordance, EX.driveable))

DeductiveClosure(Semantics).expand(g)

print(g.serialize())
#g.serialize(format="json-ld", destination="kg3.json")

#print(EX.road)