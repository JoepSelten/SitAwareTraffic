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
intersection_middle = add_has_a(g, EX.intersection, EX.middle)
intersection_road_down = add_has_a(g, EX.intersection, EX.road_down)
intersection_road_right = add_has_a(g, EX.intersection, EX.road_right)
intersection_road_up = add_has_a(g, EX.intersection, EX.road_up)
intersection_road_left = add_has_a(g, EX.intersection, EX.road_left)

g.add((intersection_middle, RDF.type, EX.middle))
g.add((intersection_road_down, RDF.type, EX.road))
g.add((intersection_road_right, RDF.type, EX.road))
g.add((intersection_road_up, RDF.type, EX.road))
g.add((intersection_road_left, RDF.type, EX.road))

road_lane_right = add_has_a(g, EX.road, EX.lane_right)
road_lane_left = add_has_a(g, EX.road, EX.lane_left)
road_side_right = add_has_a(g, EX.road, EX.side_right)
road_side_left = add_has_a(g, EX.road, EX.side_left)
road_centerline = add_has_a(g, EX.road, EX.centerline)
g.add((road_lane_right, RDF.type, EX.lane))
g.add((road_lane_right, EX.affordance, EX.waiting))
g.add((road_lane_left, RDF.type, EX.lane))
g.add((road_side_right, RDF.type, EX.side))
g.add((road_side_left, RDF.type, EX.side))
g.add((road_centerline, RDF.type, EX.centerline))


## Topology
g.add((road_lane_right, EX.connects, road_side_right))
g.add((road_lane_right, EX.connects, road_centerline))
g.add((road_lane_left, EX.connects, road_side_left))
g.add((road_lane_left, EX.connects, road_centerline))

g.add((intersection_middle, EX.connects, intersection_road_down))
g.add((intersection_middle, EX.connects, intersection_road_right))
g.add((intersection_middle, EX.connects, intersection_road_up))
g.add((intersection_middle, EX.connects, intersection_road_left))

## lower level
g.add((EX.lane, RDF.type, EX.polygon))
g.add((EX.middle, RDF.type, EX.polygon))
g.add((EX.side, RDF.type, EX.line))
g.add((EX.centerline, RDF.type, EX.line))

g.add((EX.polygon, RDF.type, EX.geometry))
g.add((EX.line, RDF.type, EX.geometry))

# polygon_interior = add_has_a(g, EX.polygon, EX.interior)
# polygon_line1 = add_has_a(g, EX.polygon, EX.line1)
# polygon_line2 = add_has_a(g, EX.polygon, EX.line2)
# polygon_line3 = add_has_a(g, EX.polygon, EX.line3)
# polygon_line4 = add_has_a(g, EX.polygon, EX.line4)

# g.add((polygon_interior, RDF.type, EX.interior))
# g.add((polygon_line1, RDF.type, EX.line))
# g.add((polygon_line2, RDF.type, EX.line))
# g.add((polygon_line3, RDF.type, EX.line))
# g.add((polygon_line4, RDF.type, EX.line))

# ## Affordances
#g.add((EX.lane, EX.affordance, EX.driveable))
#g.add((EX.side, EX.affordance, EX.perceivable))

DeductiveClosure(Semantics).expand(g)
DeductiveClosure(Semantics).expand(g)
DeductiveClosure(Semantics).expand(g)

# ## moet de uris eigenlijk queryen, dit kan miss ook uit de lagere level gehaald worden ipv los allemaal gedefinieerd
# ## nu is namelijk niet schaalbaar
# intersection_road1_lane = URIRef("http://example.com/intersection/road1/lane")
# intersection_road2_lane = URIRef("http://example.com/intersection/road2/lane")
# intersection_road3_lane = URIRef("http://example.com/intersection/road3/lane")
# intersection_road4_lane = URIRef("http://example.com/intersection/road4/lane")

# g.add((intersection_road1_lane, EX.connects, intersection_middle))
# g.add((intersection_road2_lane, EX.connects, intersection_middle))
# g.add((intersection_road3_lane, EX.connects, intersection_middle))
# g.add((intersection_road4_lane, EX.connects, intersection_middle))

# intersection_road1_lane_line1 = URIRef("http://example.com/intersection/road1/lane/line1")
# intersection_road2_lane_line1 = URIRef("http://example.com/intersection/road2/lane/line1")
# intersection_road3_lane_line1 = URIRef("http://example.com/intersection/road3/lane/line1")
# intersection_road4_lane_line1 = URIRef("http://example.com/intersection/road4/lane/line1")

# intersection_road1_lane_line2 = URIRef("http://example.com/intersection/road1/lane/line2")
# intersection_road2_lane_line2 = URIRef("http://example.com/intersection/road2/lane/line2")
# intersection_road3_lane_line2 = URIRef("http://example.com/intersection/road3/lane/line2")
# intersection_road4_lane_line2 = URIRef("http://example.com/intersection/road4/lane/line2")

# intersection_road1_lane_line3 = URIRef("http://example.com/intersection/road1/lane/line3")
# intersection_road2_lane_line3 = URIRef("http://example.com/intersection/road2/lane/line3")
# intersection_road3_lane_line3 = URIRef("http://example.com/intersection/road3/lane/line3")
# intersection_road4_lane_line3 = URIRef("http://example.com/intersection/road4/lane/line3")

# intersection_road1_lane_line4 = URIRef("http://example.com/intersection/road1/lane/line4")
# intersection_road2_lane_line4 = URIRef("http://example.com/intersection/road2/lane/line4")
# intersection_road3_lane_line4 = URIRef("http://example.com/intersection/road3/lane/line4")
# intersection_road4_lane_line4 = URIRef("http://example.com/intersection/road4/lane/line4")

# intersection_middle_line1 = URIRef("http://example.com/intersection/middle/line1")
# intersection_middle_line2 = URIRef("http://example.com/intersection/middle/line2")
# intersection_middle_line3 = URIRef("http://example.com/intersection/middle/line3")
# intersection_middle_line4 = URIRef("http://example.com/intersection/middle/line4")

# intersection_road1_side1 = URIRef("http://example.com/intersection/road1/side1")
# intersection_road1_side2 = URIRef("http://example.com/intersection/road1/side2")

# intersection_road2_side1 = URIRef("http://example.com/intersection/road2/side1")
# intersection_road2_side2 = URIRef("http://example.com/intersection/road2/side2")

# intersection_road3_side1 = URIRef("http://example.com/intersection/road3/side1")
# intersection_road3_side2 = URIRef("http://example.com/intersection/road3/side2")

# intersection_road4_side1 = URIRef("http://example.com/intersection/road4/side1")
# intersection_road4_side2 = URIRef("http://example.com/intersection/road4/side2")

# g.add((intersection_road1_lane_line1, EX.equals, intersection_middle_line1))
# g.add((intersection_road2_lane_line1, EX.equals, intersection_middle_line2))
# g.add((intersection_road3_lane_line1, EX.equals, intersection_middle_line3))
# g.add((intersection_road4_lane_line1, EX.equals, intersection_middle_line4))

# g.add((intersection_road1_side1, EX.equals, intersection_road1_lane_line2))
# g.add((intersection_road1_side2, EX.equals, intersection_road1_lane_line3))
# g.add((intersection_road2_side1, EX.equals, intersection_road2_lane_line2))
# g.add((intersection_road2_side2, EX.equals, intersection_road2_lane_line3))
# g.add((intersection_road3_side1, EX.equals, intersection_road3_lane_line2))
# g.add((intersection_road3_side2, EX.equals, intersection_road3_lane_line3))
# g.add((intersection_road4_side1, EX.equals, intersection_road4_lane_line2))
# g.add((intersection_road4_side2, EX.equals, intersection_road4_lane_line3))

# DeductiveClosure(Semantics).expand(g)

#print(g.serialize())
g.serialize(format="json-ld", destination="kg4.json")
g.serialize(destination="kg4.txt")

#print(EX.road)