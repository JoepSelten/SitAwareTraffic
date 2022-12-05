from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from rdflib.namespace import GEO
from global_variables import g
from closure_graph import Semantics
from owlrl import DeductiveClosure
EX = Namespace("http://example.com/")

def uri_from_label(label):
    query = """
        PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>

        SELECT ?subject
        WHERE {
            ?subject rdfs:label ?label
        }
        """
    answer = 0
    for r in g.query(query, initBindings={'label': Literal(label)}):
            answer = r
    
    return answer

def has_a(subject):
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

def has_sides(subject):
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

def distance_sides_road():
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

def conforms_to_area(subject):
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

def add_robot(name, *resources):
    uri = EX + URIRef(name)
    g.add((uri, RDFS.label, Literal(name)))
    g.add((uri, RDF.type, EX.robot))
    for resource in resources:
        uri_res = EX + URIRef(resource)
        g.add((uri_res, RDFS.label, Literal(resource)))
        g.add((uri, EX.resource, uri_res))
    DeductiveClosure(Semantics).expand(g)

def add_rule():
    pass

def add_affordance(name, affordance):
    uri = uri_from_label(name)
    uri_aff = EX + URIRef(affordance)
    g.add((uri_aff, RDFS.label, Literal(affordance)))
    g.add((uri, EX.affordance, uri_aff))
    

def add_resource(name, resource):
    uri = uri_from_label(name)
    uri_res = EX + URIRef(resource)
    g.add((uri_res, RDFS.label, resource))
    g.add((uri, EX.resource, uri_res))

def query_connectivity(start, goal):    # zou met zowel uris als labels moeten werken
    ## later met topology doen
    return [start, EX.middle, goal]

def query_part_of(part):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?whole
        WHERE {
            ?whole ex:has_a ?part .
        }"""

    whole = []
    for r in g.query(query, initBindings={'subject': URIRef(*part)}):
            whole.append(*r)

    return whole

def query_geometrical_location(subject):
    print(subject)
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?subject
        WHERE {
            ?subject ex:conforms_to geo:area .
        }"""

    answer = []
    for r in g.query(query, initBindings={'subject': URIRef(subject)}):
            answer.append(r)

    print(answer)       # if empty, go abstraction level higher and try again
    return answer