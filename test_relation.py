from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO
from perception import Perception

EX = Namespace("http://example.com/")
GEO = Namespace("http://www.opengis.net/ont/geosparql#")

## Creating graph
g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)

# Relation
g.add((EX.relation, RDF.type, RDFS.Class))
g.add((EX.argument, RDF.type, RDFS.Class))
g.add((EX.property, RDF.type, RDFS.Class))

g.add((EX.has_argument, RDF.type, RDF.Property))
g.add((EX.has_property, RDF.type, RDF.Property))

# is dit nodig?
g.add((EX.relation, EX.has_argument, EX.argument))
g.add((EX.relation, EX.has_property, EX.property))
g.add((EX.argument, EX.has_property, EX.property))


# Basic MereoTopological relations. zijn deze niet zo basic dat je deze gwn als predicate kunt gebruiken
g.add((EX.has_a, RDF.type, EX.relation))       # should mean that if on object, it is also on subject
g.add((EX.connects_to, RDF.type, EX.relation)) # should mean object is reachable when on subject (and also how)

# Conforms-to
g.add((EX.conforms_to, RDF.type, EX.relation)) # should mean subject conforms_to to object

## Axiomatic entity: area
area = URIRef('Area')   

## Creating new URIs 
road = URIRef('Road')
lane = URIRef('Lane')
left_side = URIRef('LeftSide')
right_side = URIRef('rightSide')

## road meta models, assuming area has known entity, to be used by software
#g.add((road, EX.conforms_to, area))
g.add((lane, EX.conforms_to, area)) 
g.add((left_side, EX.conforms_to, area)) 
g.add((right_side, EX.conforms_to, area)) 




print(g.serialize())

class TestQuery():
    def __init__(self, g):
        self.g = g
    
    def config_map(self):
        current_sit = 'Road'
        