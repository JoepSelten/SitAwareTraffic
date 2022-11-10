from rdflib import Graph, URIRef, Literal, BNode, Namespace
from rdflib.namespace import FOAF, RDF, RDFS, GEO

## Namespaces
EX = Namespace("http://example.com/")

g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)

## Relation
g.add((EX.node, RDF.type, RDFS.Class))
g.add((EX.edge, RDF.type, RDF.Property))

g.add((EX.relation, RDF.type, EX.node))
g.add((EX.property, RDF.type, EX.node))     # relations that always hold
g.add((EX.argument, RDF.type, EX.node))

g.add((EX.has_property, RDF.type, EX.edge))     # kan ik wel informatie in de edge brengen?? 
g.add((EX.has_argument, RDF.type, EX.edge))

g.add((EX.role, RDF.type, EX.property))

g.add((EX.conforms_to, RDF.type, EX.relation)) # should mean subject conforms_to to object
g.add((EX.property_conforms_to, RDF.type, EX.property))
g.add((EX.conforms_to, EX.has_property, EX.property_conforms_to))


g.add((EX.role1_conforms_to, RDF.type, EX.role))
g.add((EX.))


g.add((EX.property_conforms_to, EX.edge, EX.model))

g.add((EX.conforms_to, EX.has_argument, EX.model))      # or has_role?
g.add((EX.conforms_to, EX.has_argument, EX.metamodel))
g.add((EX.model, EX.has_property))


g.add((EX.conforms_to, EX.edg ))



print(g.serialize())