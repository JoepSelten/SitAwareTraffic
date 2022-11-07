from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO
from basic_areas import RectangleArea
from queries import uri_from_label, has_a, conforms_to_area

class Perception():
    def __init__(self):
        pass

    def recognize_sit(self, sit, side):
        self.traffic_sign = sit
        self.side = side

    def query_relations(self, g):
        uri = uri_from_label(g, self.traffic_sign)
        self.parts = has_a(g, uri)
        #query distance between sides
        query = """
        PREFIX ex: <http://example.com/>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>

        SELECT ?side
        WHERE {
            ?distance_relation ex:has_argument ?side .
            
        }"""
        
        #?distance_relation ex:has_argument ?value_distance_sides
        #?value_distance_sides rdf:value ?distance

        for r in g.query(query):
            print(r)

    def configure_map(self, g):
        road_width = 20


        for part in self.parts:
            answer = conforms_to_area(g, part)
            if answer == True:
                # query pos, yaw, length, width
                #query_side_rel_pos 
                self.plot_semantic_area(part)

    def plot_semantic_area(self, area):
        #area = RectangleArea(pos, yaw, length, width)
        #area.plot_area()
        pass