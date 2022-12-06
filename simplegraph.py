from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from closure_graph import Semantics
from owlrl import DeductiveClosure


## Namespaces
EX = Namespace("http://example.com/")
GEO = Namespace("http://www.opengis.net/ont/geosparql#")

g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)

## labels (like a traffic sign)
g.add((EX.road, RDFS.label, Literal('road')))
g.add((EX.intersection, RDFS.label, Literal('intersection')))

### Road

## Mereology
g.add((EX.road, EX.has_a, EX.lane))
g.add((EX.road, EX.has_a, EX.side1))
g.add((EX.road, EX.has_a, EX.side2))
g.add((EX.side1, RDF.type, EX.side))  # conforms to?
g.add((EX.side2, RDF.type, EX.side))

## Topology
g.add((EX.lane, EX.connects, EX.side1))         # is lane nu de argument slot die de connection aangeeft
g.add((EX.lane, EX.connects, EX.side2))

g.add((EX.road, EX.attribute, EX.width))

g.add((EX.side, EX.has_a, EX.node1))
g.add((EX.side, EX.has_a, EX.node2))

### Intersection

## Mereology
g.add((EX.intersection, EX.has_a, EX.middle))
g.add((EX.intersection, EX.has_a, EX.road1))
g.add((EX.intersection, EX.has_a, EX.road2))
g.add((EX.intersection, EX.has_a, EX.road3))
g.add((EX.intersection, EX.has_a, EX.road4))
g.add((EX.road1, RDF.type, EX.road))
g.add((EX.road2, RDF.type, EX.road))
g.add((EX.road3, RDF.type, EX.road))
g.add((EX.road4, RDF.type, EX.road))

## Topology
g.add((EX.middle, EX.connects, EX.road1))
g.add((EX.middle, EX.connects, EX.road2))  
g.add((EX.middle, EX.connects, EX.road3))  
g.add((EX.middle, EX.connects, EX.road4))  

## Geometry
g.add((EX.lane, EX.conforms_to, GEO.Area))
g.add((EX.side, EX.conforms_to, GEO.Area))
g.add((EX.middle, EX.conforms_to, GEO.Area))

## Affordances
g.add((EX.lane, EX.affordance, EX.driveable))

DeductiveClosure(Semantics).expand(g)

#print(g.serialize())
g.serialize(format="json-ld", destination="kg3.json")

#print(EX.road)