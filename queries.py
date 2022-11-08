from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO

def uri_from_label(g, label):
    query = """
        PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>

        SELECT ?subject
        WHERE {
            ?subject rdfs:label ?label
        }
        """
    for r in g.query(query, initBindings={'label': Literal(label)}):
            answer = r
    
    return answer

def has_a(g, subject):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?part
        WHERE {
            ?subject ex:has_a ?part .
        }"""

    parts = []
    for r in g.query(query, initBindings={'subject': URIRef(*subject)}):
            parts.append(*r)

    return parts

def has_sides(g, subject):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?part
        WHERE {
            ?subject ex:has_a ?part .
            ?part ex:conforms_to ex:side .
        }"""

    parts = []
    for r in g.query(query, initBindings={'subject': URIRef(*subject)}):
            parts.append(*r)

    return parts

def distance_sides_road(g):
    query = """
        PREFIX ex: <http://example.com/>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>

        SELECT DISTINCT ?distance
        WHERE {
            ?distance_relation ex:has_argument ?side .
            ?distance_relation ex:has_argument ?value_distance_sides .
            ?value_distance_sides rdf:value ?distance .
        }"""
    for r in g.query(query):
            answer = r
    return answer

def conforms_to_area(g, subject):
        query = """
        PREFIX ex: <http://example.com/>
        PREFIX geo: <http://www.opengis.net/ont/geosparql#>
        ASK {
            ?x ex:conforms_to geo:Area 
        }
        """
        for r in g.query(query, initBindings={'x': subject}):
                answer = r
        return answer



