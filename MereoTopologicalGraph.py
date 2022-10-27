from rdflib import Graph, URIRef, Literal, BNode

## Creating graph
g = Graph()

## Basic MereoTopological relations
#has_a = URIRef('has-a')
connects_to = URIRef('connects-to')
has_a = BNode()

## Labels
road = URIRef('Road')
driveable_space = URIRef('DriveableSpace')
left_side = URIRef('LeftSide')
right_side = URIRef('rightSide')

g.add((road, has_a, driveable_space))

print(g.serialize(format="json-ld"))

print(type(road))