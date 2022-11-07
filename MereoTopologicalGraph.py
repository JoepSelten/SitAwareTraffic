from operator import truediv
from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO
from gsdggs import DGGS

from rdflib.extras.external_graph_libs import rdflib_to_networkx_multidigraph
import networkx as nx
import matplotlib.pyplot as plt
import pprint

VISUALIZE = False

## Namespaces
EX = Namespace("http://example.com/")
GEO = Namespace("http://www.opengis.net/ont/geosparql#")

## Create IDs
#relation = URIRef('relation')
#has_argument = URIRef('has_argument')
#subject = URIRef('subject')
#conforms_to = URIRef('conforms_to')
#has_a = URIRef('has_a')
#connects_to = URIRef('connects_to')
#road = URIRef('road')
#lane = URIRef('lane')
#side1 = URIRef('side1')
#side2 = URIRef('side2')


## Creating graph
g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)

# Relation
g.add((EX.relation, RDF.type, RDFS.Class))
g.add((EX.has_argument, RDF.type, RDF.Property))
g.add((EX.subject, RDF.type, RDFS.Class))
g.add((EX.relation, EX.has_argument, EX.subject))


g.add((EX.has_a, RDF.type, EX.relation))       # should mean that if on object, it is also on subject
g.add((EX.connects_to, RDF.type, EX.relation)) # should mean object is reachable when on subject (and also how)

# Basic MereoTopological relations
g.add((EX.has_a, RDF.type, RDF.Property))       # should mean that if on object, it is also on subject
g.add((EX.connects_to, RDF.type, RDF.Property)) # should mean object is reachable when on subject (and also how)

# Conforms-to
g.add((EX.conforms_to, RDF.type, RDF.Property)) # should mean subject conforms_to to object

## Creating new URIs 
# road = URIRef('Road')
# lane = URIRef('DriveableSpace')
# side1 = URIRef('LeftSide')
# side2 = URIRef('rightSide')

## classes
g.add((EX.road, RDF.type, RDFS.Class))      # of zijn dit subjects en relations?
g.add((EX.lane, RDF.type, RDFS.Class))
g.add((EX.side1, RDF.type, RDFS.Class))
g.add((EX.side2, RDF.type, RDFS.Class))

# labels
g.add((EX.road, RDFS.label, Literal('road')))

## road meta models, assuming area has known entity, to be used by software
g.add((EX.road, EX.conforms_to, GEO.Area)) # GEO.Geometry is a subclass of GEO.SpatialObject
g.add((EX.lane, EX.conforms_to, GEO.Area)) 
g.add((EX.side1, EX.conforms_to, GEO.Area)) 
g.add((EX.side2, EX.conforms_to, GEO.Area)) 

# MereoTopological graph
g.add((EX.road, EX.has_a, EX.lane))
g.add((EX.road, EX.has_a, EX.side1))
g.add((EX.road, EX.has_a, EX.side2))
g.add((EX.lane, EX.connects_to, EX.side1))
g.add((EX.lane, EX.connects_to, EX.side2))

# Meta model
#g.add((lane, EX.conforms_to, GEO.SpatialObject))     # or GEO.Geometry, which is a subclass of GEO.SpatialObject
# g.add((lane, EX.conforms_to, GEO.Geometry))
# g.add((side1, EX.conforms_to, GEO.Geometry))
# g.add((side2, EX.conforms_to, GEO.Geometry))

# # Geometric relations
g.add((EX.Distance_sides, RDF.type, EX.relation))     # still conceptually
g.add((EX.Distance_sides, EX.has_argument, EX.side2))
g.add((EX.Distance_sides, EX.has_argument, EX.side1))
g.add((EX.Distance_sides, EX.has_argument, EX.value_distance_sides))
g.add((EX.value_distance_sides, RDF.value, Literal(20)))

#g.add((EX.RelativePosition, RDF.type, EX.relation))     # still conceptually
#g.add((EX.Value, RDF.type, EX.argument))
#g.add((EX.RelativePostion, EX.has_argument, EX.Value))

# g.add((EX.RelativePosition, RDF.type, RDF.Property))
# g.add((EX.ValueRelativePostion, RDF.type, RDF.Property))

# g.add((EX.RelativePostion1, RDF.type, EX.RelativePosition))

# g.add((EX.lane, EX.RelativePostion, EX.side1))
# g.add((EX.lane, EX.ValueRelativePosition, Literal(1)))

#g.add((lane, GEO.asDGGS, Literal('CELLLIST ((R0 R10 R13 R16 R30 R31 R32 R40))')))
#g.add((side1, GEO.asDGGS, Literal('CELLLIST ((R06 R07 R30 R31))')))

g.serialize(format="json-ld", destination="kg.json")

q = """
    PREFIX ex: <http://example.com/>
            
    SELECT ?part
    WHERE {
        ex:road ex:has_a ?part .
    }"""

p = """
    PREFIX geo: <http://www.opengis.net/ont/geosparql#>
    PREFIX dggs: <https://placeholder.com/dggsfuncs/>
    
    SELECT ?a ?b 
    WHERE {
        ?a geo:asDGGS ?a_geom .
        ?b geo:asDGGS ?b_geom .
        
        FILTER dggs:sfWithin(?a_geom, ?b_geom)
    }"""


# Interate through and print results
#print(g.query(p))
# for stmt in g:
#     pprint.pprint(stmt)


#for r in g.query(q):
    #print(q)
    #print(r)
    

if VISUALIZE:
    result = g 
    G = rdflib_to_networkx_multidigraph(result)
    pos = nx.spring_layout(G, scale=2)
    edge_labels = nx.get_edge_attributes(G, 'r')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    nx.draw(G, with_labels=True)
    plt.show()