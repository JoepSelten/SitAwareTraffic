from operator import truediv
from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF
from rdflib.namespace import GEO
from rdflib.extras.external_graph_libs import rdflib_to_networkx_multidigraph
import networkx as nx
import matplotlib.pyplot as plt

VISUALIZE = False

## Namespaces
EX = Namespace("http://example.com/")
GEO = Namespace("http://www.opengis.net/ont/geosparql#")

## Creating graph
g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)

# Basic MereoTopological relations
g.add((EX.has_a, RDF.type, RDF.Property))
g.add((EX.connects_to, RDF.type, RDF.Property))
#EX.has_a = URIRef('has-a')
#EX.connects_to = URIRef('connects-to')
#has_a = BNode()

## Labels 
road = URIRef('Road')
driveable_space = URIRef('DriveableSpace')
left_side = URIRef('LeftSide')
right_side = URIRef('rightSide')

# MereoTopological graph
g.add((road, EX.has_a, driveable_space))
g.add((road, EX.has_a, left_side))
g.add((road, EX.has_a, right_side))
g.add((driveable_space, EX.connects_to, left_side))
g.add((driveable_space, EX.connects_to, right_side))

# Meta model
g.add((EX.conforms_to, RDF.type, RDF.Property))
#g.add((driveable_space, EX.conforms_to, GEO.SpatialObject))     # or GEO.Geometry, which is a subclass of GEO.SpatialObject
g.add((driveable_space, RDF.type, GEO.Geometry))
g.add((left_side, RDF.type, GEO.Geometry))
g.add((right_side, EX.conforms_to, GEO.SpatialObject))

# Geometric relations
g.add((driveable_space, GEO.sfTouches, left_side))
g.add((driveable_space, GEO.sfTouches, right_side))

# Geometric properties
# g.add((driveable_space, GEO.asGML, Literal("<gml:Point gml:id='p21' srsName='http://www.opengis.net/def/crs/EPSG/0/4326'> \
#     <gml:coordinates>45.67, 88.56</gml:coordinates> \
#  </gml:Point>", datatype=GEO.gmlLiteral))) 

# g.add((left_side, GEO.asGML, Literal("<gml:Point gml:id='p22' srsName='http://www.opengis.net/def/crs/EPSG/0/4327'> \
#     <gml:coordinates>55.67, 88.56</gml:coordinates> \
#  </gml:Point>", datatype=GEO.gmlLiteral)))

# g.add((driveable_space, GEO.asGML, GEO.gmlLiteral))) 

# g.add((left_side, GEO.asGML, Literal("<gml:Point gml:id='p21' srsName='http://www.opengis.net/def/crs/EPSG/0/4326'> \
#     <gml:coordinates>55.67, 88.56</gml:coordinates> \
#  </gml:Point>")))

g.add((driveable_space, GEO.asGML, Literal( 
    '''{"type": "Point", "coordinates": [-83.38,33.95]}''', 
    datatype=GEO.geoJSONLiteral))) 

g.add((left_side, GEO.asGML, Literal(
    '''{"type": "Point", "coordinates": [-93.38,33.95]}''',
    datatype=GEO.geoJSONLiteral)))

#print(g.serialize(format="json-ld"))
#print(g.serialize())

#print(type(road))


# Query the in-memory graph
q = """
    PREFIX geo: <http://www.opengis.net/ont/geosparql#>
    PREFIX geof: </req/geometry-extension/query-functions>
    
    SELECT geof:distance(?a_geom, ?b_geom)
    WHERE {
        ?a geo:asGML ?a_geom .
        ?b geo:asGML ?b_geom .
    }"""

p = """
    PREFIX geo: <http://www.opengis.net/ont/geosparql#>
    PREFIX geof: <http://www.opengis.net/def/function/geosparql/>

    SELECT ?what
    WHERE {
    ?what geo:hasGeometry ?geometry .

    FILTER(geof:sfWithin(?geometry,
        "POLYGON((-77.089005 38.913574,-77.029953 38.913574,-77.029953 38.886321,-77.089005 38.886321,-77.089005 38.913574))"^^geo:wktLiteral))
}"""

# Interate through and print results
for r in g.query(q):
    print(r)



if VISUALIZE:
    result = g 
    G = rdflib_to_networkx_multidigraph(result)
    pos = nx.spring_layout(G, scale=2)
    edge_labels = nx.get_edge_attributes(G, 'r')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    nx.draw(G, with_labels=True)
    plt.show()