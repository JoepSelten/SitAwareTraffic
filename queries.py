from rdflib import Graph, URIRef, Literal, BNode, Namespace, RDF, RDFS
from global_variables import g, EX, GEO
from closure_graph import Semantics
from owlrl import DeductiveClosure

def query_is_on(g, robot):
    query = """
        PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        PREFIX ex: <http://example.com/>

        SELECT ?pos
        WHERE {
            ?robot ex:is_on ?pos
        }
        """
    answer = 0
    for r in g.query(query, initBindings={'robot': robot}):
        answer = r[0]
    
    return answer

def query_type(g, subject):
    query = """
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
        PREFIX ex: <http://example.com/>

        SELECT ?object
        WHERE {
            ?subject rdf:type ?object .
            ?object rdf:type ex:geometry .
            ?object rdf:type ex:polygon .
        }
        """
    answer = 0
    for r in g.query(query, initBindings={'subject': subject}):
        answer = r[0]
    
    return answer

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
        answer = r[0]
    
    return answer

def has_a(subject):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?part
        WHERE {
            ?subject ex:has_a ?part .
        }"""

    parts = []
    for r in g.query(query, initBindings={'subject': URIRef(subject)}):
        parts.append(r[0])

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
        parts.append(r[0])

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

def query_one_link_connection(start, goal):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?connection
        WHERE {
            ?start ex:connects ?connection .
            ?connection ex:connects ?goal .
        }"""

    connection = []
    for r in g.query(query, initBindings={'start': start, 'goal': goal}):
        connection.append(r[0])
        
    return connection[0]

def query_if_direct_connected(start, goal):
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?start ex:connects ?goal }
        """

    for r in g.query(query, initBindings={'start': start, 'goal': goal}):
        answer = r

    return answer

def query_if_connected(start, goal):
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?start ex:connects* ?goal }
        """

    for r in g.query(query, initBindings={'start': start, 'goal': goal}):
        answer = r

    return answer


def query_connectivity(start, goal):    # zou met zowel uris als labels moeten werken
    is_connected = query_if_connected(start, goal)
    if is_connected:      ## beetje beun manier maargoed, moet eigenlijk met een bepaalde search (miss indoorgml)
        is_direct_connected = query_if_direct_connected(start, goal)
        if is_direct_connected:
            return [start, goal]
        else:
            one_link_connection = query_one_link_connection(start, goal)
            if one_link_connection:
                return [start, one_link_connection, goal]
            if not one_link_connection:
                #print("There is a connection but I cannot find it!")
                return False
    else:
        #print("NO connection found!")
        return is_connected

def query_part_of(part):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT DISTINCT ?whole
        WHERE {
            ?whole ex:has_a ?part .
        }"""

    whole = []
    for r in g.query(query, initBindings={'part': URIRef(part)}):
        whole.append(r[0])

    return whole[0]

def simple_check(subject, predicate, object):
    query = """
        PREFIX ex: <http://example.com/>
        PREFIX geo: <http://www.opengis.net/ont/geosparql#>

        SELECT ?subject
        WHERE {
            ?subject ?predicate ?object .
        }"""
    answer = []
    for r in g.query(query, initBindings={'subject': URIRef(subject), 'predicate': URIRef(predicate), 'object': URIRef(object)}):
        answer.append(r[0])
    # if empty, go abstraction level higher and try again
    return answer

def query_check_conforms_to(subject, object):
    return simple_check(subject, EX.conforms_to, object)

def query_check_affordance(subject, object):
    return simple_check(subject, EX.affordance, object)

def query_check_geometrical(subject):
    return query_check_conforms_to(subject, GEO.Area)

def query_check_driveable(subject):
    return query_check_affordance(subject, EX.driveable)

# def query_type(subject):
#     query = """
#         PREFIX ex: <http://example.com/>

#         SELECT ?object
#         WHERE {
#             ?subject rdf:type ?object .
#         }"""
#     answer = []
#     for r in g.query(query, initBindings={'subject': URIRef(subject)}):
#         answer.append(r[0])
#     # if empty, go abstraction level higher and try again
#     return answer

def query_driveable_part(parts):
    for part in parts:
        answer = query_check_driveable(part)
        if answer:
            return part

def query_driveable_location(subject):
    drive_loc = query_check_geometrical(subject)
    n = 0
    while not drive_loc:
        parts = has_a(subject)
        for part in parts:
            geom_part = query_check_geometrical(part)
            if not geom_part:
                continue
            drive_part = query_check_driveable(geom_part[0])
            if drive_part:
                drive_loc.append(drive_part[0])
        n+=1
        if n > 3:   # for preventing an infinite loop
            print("NO GEOMETRICAL AREA FOUND")
            break
    return drive_loc[0]

def query_plan(connectivity):
    plan = []
    for i in range(len(connectivity)-1):    # dit doe je eigenlijk ook al met alleen start en goal
        action = query_action(connectivity[i], connectivity[i+1])
        plan.append(action)
    return plan

def query_action(pre, post):
    action = "move"
    return action

def query_mereology(goal, current_area):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?whole1
        WHERE {
            ?meta_whole ex:has_a ?whole2 .
            ?meta_whole ex:has_a ?whole1 .
            ?whole1 ex:has_a ?part .
        }"""

    whole = []
    for r in g.query(query, initBindings={'part': current_area, 'whole2': goal}):
        whole.append(r[0])

    return whole[0]
    

def query_line_connection(start, goal):    # dit is ook een beetje n beun manier, maar je kunt wel beargumenteren dat dit wel composable is omdat engineers van databases zo n dingen kunnen maken/oplossen
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?part1 ?part2
        WHERE {
            ?start ex:has_a ?part1 .
            ?goal ex:has_a ?part2 .
            ?part1 rdf:type ex:line .
            ?part2 rdf:type ex:line .
            ?part1 ex:equals ?part2 .

        }"""

    connection = []
    for r in g.query(query, initBindings={'start': start, 'goal': goal}):
        connection.append(r[0])
        connection.append(r[1])

    return connection

def query_if_polygon(subject):    # zou met zowel uris als labels moeten werken
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?subject rdf:type ex:polygon }
        """

    for r in g.query(query, initBindings={'subject': subject}):
        answer = r

    return answer

def query_if_line(subject):    # zou met zowel uris als labels moeten werken
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?subject rdf:type ex:line }
        """

    for r in g.query(query, initBindings={'subject': subject}):
        answer = r

    return answer

def query_common_whole(start, goal):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?whole
        WHERE {
            ?whole ex:has_a* ?start .
            ?whole ex:has_a* ?goal .
        }"""

    whole = []
    for r in g.query(query, initBindings={'start': start, 'goal': goal}):
        whole.append(r[0])

    return whole[0]

def query_topology(whole, start, goal):    # zou met zowel uris als labels moeten werken
    is_connected = query_if_connected(start, goal)
    if is_connected:      ## beetje beun manier maargoed, moet eigenlijk met een bepaalde search (miss indoorgml)
        is_direct_connected = query_if_direct_connected(start, goal)
        if is_direct_connected:
            return [start, goal]
        else:
            one_link_connection = query_one_link_connection(start, goal)
            if one_link_connection:
                return [start, one_link_connection, goal]
            if not one_link_connection:
                #print("There is a connection but I cannot find it!")
                return False
    else:
        #print("NO connection found!")
        return is_connected

def query_if_perceivable(subject):    # zou met zowel uris als labels moeten werken
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?subject ex:affordance ex:perceivable }
        """

    for r in g.query(query, initBindings={'subject': subject}):
        answer = r

    return answer

def query_geom_type(subject):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?object
        WHERE {
            ?subject rdf:type ?object .
            ?object rdf:type ex:geometry .
        }"""
    answer = []
    for r in g.query(query, initBindings={'subject': URIRef(subject)}):
        answer.append(r[0])
    # if empty, go abstraction level higher and try again
    return answer

def query_equivalance(part):
    polygon_uri = URIRef("http://example.com/polygon")
    line_uri = URIRef("http://example.com/line")
    is_line = query_if_line(part)
    part_type = query_type(part)
    if polygon_uri in part_type:
        print(f'part: {part}')
    #print(part_type)
    return part_type

def query_if_equal(subject, object):
    query = """
        PREFIX ex: <http://example.com/>

        ASK { ?subject ex:equals ?object }
        """

    for r in g.query(query, initBindings={'subject': subject, 'object': object}):
        answer = r

    return answer

def query_left_part(subjects):
    answer = 0
    for subject in subjects:
        if subject[-1] == str(1):
            answer = subject
    return answer

def query_right_part(subjects):
    answer = 0
    for subject in subjects:
        if subject[-1] == str(2):
            answer = subject
    return answer

def has_geometries(subject):
    query = """
        PREFIX ex: <http://example.com/>

        SELECT ?part
        WHERE {
            ?subject ex:has_a* ?part .
            ?part rdf:type ex:geometry
        }"""

    parts = []
    for r in g.query(query, initBindings={'subject': URIRef(subject)}):
        parts.append(r[0])

    return parts

def query_if_first_layer_geometry(subject):    # zou met zowel uris als labels moeten werken
    query = """
        PREFIX ex: <http://example.com/>

        ASK {
            ?whole ex:has_a ?subject .
            ?whole rdf:type ex:geometry
            }
        """
    answer = True
    for r in g.query(query, initBindings={'subject': subject}):
        if r:
            answer = False

    return answer

def has_first_layer_geometries(subject):
    geometries = has_geometries(subject)
    parts = []
    for geometry in geometries:
        if query_if_first_layer_geometry(geometry):
            parts.append(geometry)
    return parts