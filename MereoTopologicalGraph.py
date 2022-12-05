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

class Relation():
    def __init__(self, label):
        self.properties = []
        self.label = label

    def add_property(self, property):
        self.properties.append(property)

    def conforms_to(self, metamodel, properties):   # is miss niet nodig
        for props in metamodel.properties:
            if props in properties:
                self.properties.append(props)


class has_a(Relation):
    def __init__(args, g, self):
        super().__init__()
        g.add((EX.has_a, RDF.type, EX.relation))
        g.add((EX.has_a, EX.has_property, EX.arg1))

       

class PropertyGraph():
    def __init__(self):
        self.g = Graph()
        self.g.bind('ex', EX)
        self.add_axioms()

    def add_axioms(self):
        ## Basic graph structures
        self.g.add((EX.node, RDF.type, RDFS.Class))
        self.g.add((EX.edge, RDF.type, RDF.Property))

        ## Implicit relations, assumed to be known by humans, is therefore represented as an edge in the KG
        self.g.add((EX.has_a, RDF.type, EX.edge))    # has_a is the highest level of abstraction: mereological relation. 
        self.g.add((EX.connects, RDF.type, EX.edge)) # connects and connects already provide some additional information: topological relations
        self.g.add((EX.contains, RDF.type, EX.edge)) # you can connect an argument to a role

        self.g.add((EX.conforms_to, RDF.type, EX.edge))  # conforms_to is a less restrictive form of the is_a relation

        # Property relation, implicit when talking about property graphs, but needed for reification for RDF triples
        self.g.add((EX.has_property, RDF.type, EX.edge))

        ## Explicit relations, uses the implicit relations as building blocks
        self.g.add((EX.relation, RDF.type, EX.node))
        self.g.add((EX.property, RDF.type, EX.node))     # zijn dit laatste drie nodig? Kan ik niet alles als een relatie modeleren
        self.g.add((EX.argument, RDF.type, EX.node))
        self.g.add((EX.entity, RDF.type, EX.node))

    def add_relation(self, relation):
        pass
        
    def area(self, label='area'):
        self.g.add((EX.area, RDF.type, EX.entity))
        self.g.add((EX.length, RDF.type, EX.attribute))
        self.g.add((EX.width, RDF.type, EX.attribute))
        self.g.add((EX.pos, RDF.type, EX.attribute))
        self.g.add((EX.orientation, RDF.type, EX.attribute))

    def lane(self):
        self.g.add((EX.lane, RDF.type, EX.relation))
        self.g.add((EX.lane, RDFS.label, Literal('lane')))

        self.area(EX.area1, EX.lane)
        self.line(EX.line1, EX.lane)

        self.g.add((EX.lane, EX.connects, EX.area1))
        self.g.add((EX.lane, EX.connects, EX.line1))

    def point(self, new_point, value_pos):
        self.g.add((new_point, RDF.type, EX.point))
        self.g.add((EX.point, RDF.type, EX.entity))
        self.g.add((EX.pos, RDF.type, EX.relation))      # of is dit n property?
        self.g.add((EX.point, EX.has_property, EX.pos))  # dit is een relative pos, heeft vgm geen zin om n frame te defineren en hier een vector voor te pakken


        #g.add((EX.pos, EX.connects, EX.start_vec))          # is de input van een property ook een connects relatie?
        #g.add((EX.pos, EX.connects, EX.end_vec)) 

        #self.g.add((EX.value_pos, RDF.type, EX.relation))
        #self.g.add((EX.pos, EX.has_property, EX.value_pos))

    def pos(self):
        self.g.add((EX.pos, RDF.type, EX.relation))
        self.g.add((EX.value_pos, RDF.type, EX.attribute))
        self.g.add((EX.pos, EX.has_attribute, EX.value_pos))

    def line(self, new_line):
        self.g.add((new_line, RDF.type, EX.line))
        self.g.add((EX.line, RDF.type, EX.relation))
        #self.g.add((EX.orientation, RDF.type, EX.relation))
        #self.g.add((EX.length, RDF.type, EX.relation))
        self.g.add((EX.line, EX.has_property, EX.orientation))
        self.g.add((EX.line, EX.has_property, EX.length))

        self.point(EX.point1, EX.line)
        self.point(EX.point2, EX.line)

        #self.g.add((EX.point1, RDF.type, EX.point))
        #self.g.add((EX.point2, RDF.type, EX.point))

        self.g.add((EX.line, EX.connects, EX.point1))
        self.g.add((EX.line, EX.connects, EX.point2))

    def orientation(self):
        self.g.add((EX.orientation, RDF.type, EX.relation))
        self.g.add((EX.orientation, EX.property_of, EX.line))


    def corner(self):
        ## connects two lines
        self.g.add((EX.corner, RDF.type, EX.relation))

        #self.g.add((EX.angle, RDF.type, EX.relation))

        #self.g.add((EX.shares_point, RDF))
        
        self.g.add((EX.corner, EX.has_property, EX.angle))
        self.g.add((EX.corner, EX.has_property, EX.sharing_point))


        self.line(EX.line1)
        self.line(EX.line2)

        #query_point(EX.line1)   # zoiets miss


        ## hoe link ik twee points van de lijnen

        #self.g.add((EX.line1, RDF.type, EX.line))
        #self.g.add((EX.line2, RDF.type, EX.line))

        self.g.add((EX.corner, EX.connects, EX.line1))
        self.g.add((EX.corner, EX.connects, EX.line2))

    def sharing_point(self):
        self.g.add((EX.sharing_point, RDF.type, EX.relation))

        query_corner = '''
        PREFIX ex: <http://example.com/>
        SELECT ?corner
        WHERE {
            ?corner ex:connects ex:line1 .
            ?corner ex:connects ex:line2 .
        }
        '''
        for r in self.g.query(query_corner):
            print(r)
            corner = r

        query = """
        PREFIX ex: <http://example.com/>
        PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
        SELECT ?pos1
        WHERE {
            ?corner ex:connects ?line1 .
            ?line1 rdf:type ?line .
            ?line ex:connects ?point2 .
            ?point2 ex:has_property ?pos1 .
        }
        """
        #for r in self.g.query(query, initBindings={'corner': URIRef(*corner)}):
            #print(r)
        for r in self.g.query(query):
            print(r)

    
    def add_relation(self):
        pass


    def rules(self):
        pass
    

pg = PropertyGraph()
pg.corner()
print(pg.g.serialize())
pg.sharing_point()

'''
## Creating graph
g = Graph()
g.bind('ex', EX)
g.bind('geo', GEO)


## Basic graph structures
g.add((EX.node, RDF.type, RDFS.Class))
g.add((EX.edge, RDF.type, RDF.Property))

## Implicit relations, assumed to be known by humans, is therefore represented as an edge in the KG
g.add((EX.has_a, RDF.type, EX.edge))    # has_a is the highest level of abstraction: mereological relation. 
g.add((EX.connects, RDF.type, EX.edge)) # connects and connects already provide some additional information: topological relations
g.add((EX.contains, RDF.type, EX.edge)) # you can connect an argument to a role

g.add((EX.conforms_to, RDF.type, EX.edge))  # conforms_to is a less restrictive form of the is_a relation

# Property relation, implicit when talking about property graphs, but needed for reification for RDF triples
g.add((EX.has_property, RDF.type, EX.edge))

## Explicit relations, uses the implicit relations as building blocks
g.add((EX.relation, RDF.type, EX.node))
g.add((EX.property, RDF.type, EX.node))     # zijn dit laatste drie nodig? Kan ik niet alles als een relatie modeleren
g.add((EX.argument, RDF.type, EX.node))
g.add((EX.entity, RDF.type, EX.node))


#g.add((EX.relation, EX.has_a, EX.entity))

# g.add((EX.has_property, RDF.type, EX.edge))
# g.add((EX.has_argument, RDF.type, EX.edge))


#g.add((EX.role, RDF.type, EX.property))

#g.add((EX.subject, RDF.type, RDFS.Class))
#g.add((EX.relation, EX.edge, EX.subject))


## Basic MereoTopological relations
#g.add((EX.has_a, RDF.type, RDF.Property))       # should mean that if on object, it is also on subject
#g.add((EX.connects, RDF.type, RDF.Property)) # should mean object is reachable when on subject (and also how)

# or 
#g.add((EX.has_a, RDF.type, EX.relation))       # should mean that if on object, it is also on subject
#g.add((EX.connects, RDF.type, EX.relation)) # should mean object is reachable when on subject (and also how)

## Conforms-to
#g.add((EX.conforms_to, RDF.type, RDF.Property)) # should mean subject conforms_to to object

# or 
#g.add((EX.conforms_to, RDF.type, EX.relation)) # should mean subject conforms_to to object
#g.add((EX.property_conforms_to, RDF.type, EX.property))
#g.add((EX.conforms_to, EX.has_property, EX.property_conforms_to))

#g.add((EX.role1_conforms_to, RDF.type, EX.role))


g.add((EX.property_conforms_to, EX.edge, EX.model))

g.add((EX.conforms_to, EX.edge, EX.model))
g.add((EX.conforms_to, EX.edge, EX.metamodel))


## Creating new URIs 
# road = URIRef('Road')
# lane = URIRef('DriveableSpace')
# side1 = URIRef('LeftSide')
# side2 = URIRef('rightSide')

## classes
g.add((EX.road, RDF.type, EX.relation))      # of zijn dit subjects/enities en relations?
g.add((EX.lane, RDF.type, EX.relation))
#g.add((EX.side1, RDF.type, RDFS.Class))
#g.add((EX.side2, RDF.type, RDFS.Class))

# or
g.add((EX.side, RDF.type, EX.relation))
g.add((EX.side1, EX.conforms_to, EX.side))
g.add((EX.side2, EX.conforms_to, EX.side))

# or and
g.add((EX.road1, EX.conforms_to, EX.road))
g.add((EX.lane1, EX.conforms_to, EX.lane))

# or 
g.add((EX.conforms_to, EX.road1, EX.road))

## labels
g.add((EX.road, RDFS.label, Literal('road')))
g.add((EX.lane, RDFS.label, Literal('lane')))
g.add((EX.side1, RDFS.label, Literal('side1')))
g.add((EX.side2, RDFS.label, Literal('side2')))

## road meta models, assuming area has known entity, to be used by software
g.add((EX.road, EX.conforms_to, GEO.Area)) # GEO.Geometry is a subclass of GEO.SpatialObject
g.add((EX.lane, EX.conforms_to, GEO.Area)) 
#g.add((EX.side1, EX.conforms_to, GEO.Area)) 
#g.add((EX.side2, EX.conforms_to, GEO.Area))

# or
g.add((EX.side, EX.conforms_to, GEO.Area))

## MereoTopological graph
g.add((EX.road, EX.has_a, EX.lane))
g.add((EX.road, EX.has_a, EX.side1))
g.add((EX.road, EX.has_a, EX.side2))

## point
g.add((EX.point, RDF.type, EX.entity))

g.add((EX.pos, RDF.type, EX.relation))      # of is dit n property?
g.add((EX.point, EX.has_property, EX.pos))  # dit is een relative pos, heeft vgm geen zin om n frame te defineren en hier een vector voor te pakken

#g.add((EX.pos, EX.connects, EX.start_vec))          # is de input van een property ook een connects relatie?
#g.add((EX.pos, EX.connects, EX.end_vec)) 

g.add((EX.value_pos, RDF.type, EX.relation))
g.add((EX.pos, EX.has_property, EX.value_pos))

## line
g.add((EX.line, RDF.type, EX.relation))
g.add((EX.line, EX.connects, EX.point1))
g.add((EX.line, EX.connects, EX.point2))

g.add((EX.point1, RDF.type, EX.point))
g.add((EX.point2, RDF.type, EX.point))

g.add((EX.pos_point1, RDF.type, EX.pos))
g.add((EX.pos_point2, RDF.type, EX.pos))

g.add((EX.value_pos_point1, RDF.type, EX.value_pos1))
g.add((EX.value_pos_point2, RDF.type, EX.value_pos2))

g.add((EX.point1, EX.has_property, EX.pos_point1))          # instead of redoing this, can I reason that it should inherit the properties of point
g.add((EX.pos_point1, EX.has_property, EX.value_pos_point1))    # hoe refereer ik in dat geval naar de positie van een specifiek punt

g.add((EX.point2, EX.has_property, EX.pos_point2))
g.add((EX.pos_point2, EX.has_property, EX.value_pos_point2))

g.add((EX.length, RDF.type, EX.relation))
g.add((EX.orientation, RDF.type, EX.relation))

g.add((EX.line, EX.has_property, EX.length))        # met welke properties zou ik de points automatisch kunnen definieren
g.add((EX.line, EX.has_property, EX.orientation))   # moet eigenlijk met een point en deze eigenschappen de andere kunnen bepalen

g.add((EX.length, EX.connects, EX.pos_point1))             # of moet hier de value_pos_point1 staan?
g.add((EX.length, EX.connects, EX.pos_point2))          # uiteindelijk wijst het naar een wiskundige formule?
g.add((EX.length, EX.has_property, EX.value_length))



## corner
g.add((EX.corner, RDF.type, EX.relation))
g.add((EX.corner, EX.connects, EX.line1))
g.add((EX.corner, EX.connects, EX.line2))

g.add((EX.line1, RDF.type, EX.line))
g.add((EX.line2, RDF.type, EX.line))

g.add((EX.length1, RDF.type, EX.length))
g.add((EX.length2, RDF.type, EX.length))

g.add(EX.line1, EX.has_property, EX.length1)
g.add(EX.line2, EX.has_property, EX.length2)





g.add((EX.lane, EX.connects, EX.side1))         # is lane nu de argument slot die de connection aangeeft
g.add((EX.lane, EX.connects, EX.side2))

g.add((EX.lane, EX.has_property, EX.distance))

g.add((EX.side, EX.has_property, EX.length))


#g.add((EX.has_a, EX.road, EX.lane))
#g.add((EX.has_a, EX.road, EX.side1))
#g.add((EX.has_a, EX.intersection, EX.traffic_light))

#g.add((EX.connects, ))

# Meta model
#g.add((lane, EX.conforms_to, GEO.SpatialObject))     # or GEO.Geometry, which is a subclass of GEO.SpatialObject
# g.add((lane, EX.conforms_to, GEO.Geometry))
# g.add((side1, EX.conforms_to, GEO.Geometry))
# g.add((side2, EX.conforms_to, GEO.Geometry))

## How can it be perceived?

g.add((EX.perceive_side, RDF.type, EX.relation))
g.add((EX.perceive_side, EX.edge, EX.side))
g.add((EX.perceive_side, EX.edge, EX.laser_scanner))

# or?
#g.add((EX.perceive, RDF.type, EX.relation))
#g.add((EX.perceive, EX.edge, EX))

## Geometric relations
g.add((EX.Distance_sides, RDF.type, EX.relation))     # still conceptually
g.add((EX.Distance_sides, EX.edge, EX.side2))
g.add((EX.Distance_sides, EX.edge, EX.side1))

#g.add((EX.Distance_sides, EX.edge, EX.value_distance_sides))
#g.add((EX.value_distance_sides, RDF.value, Literal(20)))

#or
#g.add((EX.side1, EX.Distance_sides, EX.side2))

# or
g.add((EX.value_distance_sides, RDF.type, EX.relation))
g.add((EX.value_distance_sides, EX.edge, EX.Distance_sides))
g.add((EX.value_distance_sides, EX.edge, Literal(20)))

## task
g.add((EX.task, RDF.type, EX.relation))
g.add((EX.task_road, EX.conforms_to, EX.task))
g.add((EX.task_road, EX.edge, EX.road))
g.add((EX.task_road, EX.edge, EX.drive_straight))

#g.add((EX.drive))

## behaviour
g.add((EX.behaviour, RDF.type, EX.relation))

g.add((EX.behaviour_lane, EX.conforms_to, EX.behaviour))
g.add((EX.behaviour_lane, EX.edge, EX.lane))
g.add((EX.behaviour_lane, EX.edge, EX.drivable_space))

#g.add((EX.drivable_space, RDF.type, EX.relation))
#g.add((EX.drivable_space, EX.edge, ))

g.add((EX.behaviour_side, EX.conforms_to, EX.behaviour))
g.add((EX.behaviour_side, EX.edge, EX.side))
g.add((EX.behaviour_side, EX.edge, EX.no_enter))

# or affordances
g.add((EX.affordance, RDF.type, EX.relation))

g.add((EX.affordance_lane, EX.conforms_to, EX.affordance))
g.add((EX.affordance_lane, EX.edge, EX.lane))
g.add((EX.affordance_lane, EX.edge, EX.drivable_space))

## skills/resources
g.add((EX.resource, RDF.type, EX.relation))

g.add((EX.resource_velocity_control, EX.conforms_to, EX.resource))
g.add((EX.resource_velocity_control, EX.edge, EX.robot))
g.add((EX.resource_velocity_control, EX.edge, EX.velocity_control))

g.add((EX.skill, RDF.type, EX.relation))
g.add((EX.skill_move_on_road, EX.conforms_to, EX.skill))
#g.add((EX.skill_move_on_road, EX.edge, EX.requirements))

g.add((EX.requirements, RDF.type, EX.relation))
g.add((EX.requirements_move_on_road, EX.conforms_to, EX.requirements))
g.add((EX.requirements_move_on_road, EX.edge, EX.velocity_control))
g.add((EX.requirements_move_on_road, EX.edge, EX.requirements))

#or
# g.add((EX.resource, RDF.type, EX.relation))
# g.add((EX.resource, EX.edge, EX.robot))

# g.add((EX.resource_vel_control, EX.conforms_to, EX.resource))
# g.add((EX.resource_vel_control, EX.edge, EX.velocity_control))

g.add((EX.driving_vel_control, RDF.type, EX.relation))
g.add((EX.driving_vel_control, EX.edge, EX.drivable_space))
g.add((EX.driving_vel_control, EX.edge, EX.velocity_control))

g.add((EX.actions, RDF.type, EX.relation))

g.add((EX.resource, RDF.type, EX.relation))
g.add((EX.resource, EX.edge, EX.robot))
g.add((EX.resource, EX.edge, EX.laser_scanner))





g.serialize(format="json-ld", destination="kg2.json")

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

'''