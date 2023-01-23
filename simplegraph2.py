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

#g.add((EX.lane_following, EX.skill, EX.turn_drive))

## Mereology
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

g.serialize(destination="reportgraph.txt")

#print(EX.road)