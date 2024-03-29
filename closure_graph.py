import rdflib
from rdflib import Literal, Namespace, URIRef
from rdflib.namespace import RDF, RDFS
from itertools import product
from owlrl.Closure import Core
from owlrl.AxiomaticTriples import RDFS_Axiomatic_Triples, RDFS_D_Axiomatic_Triples

EX = Namespace("http://example.com/")
GEO = Namespace("http://www.opengis.net/ont/geosparql#")

class Semantics(Core):
    """
    RDFS Semantics class, ie, implementation of the RDFS closure graph.

    .. note:: Note that the module does *not* implement the so called Datatype entailment rules, simply because the
        underlying RDFLib does not implement the datatypes (ie, RDFLib will not make the literal "1.00" and "1.00000"
        identical, although even with all the ambiguities on datatypes, this I{should} be made equal...).

        Also, the so-called extensional entailment rules (Section 7.3.1 in the RDF Semantics document) have not been
        implemented either.

    The comments and references to the various rule follow the names as used in the `RDF Semantics document`_.

    .. _RDF Semantics document: http://www.w3.org/TR/rdf-mt/

    :param graph: The RDF graph to be extended.
    :type graph: :class:`rdflib.Graph`

    :param axioms: Whether (non-datatype) axiomatic triples should be added or not.
    :type axioms: bool

    :param daxioms: Whether datatype axiomatic triples should be added or not.
    :type daxioms: bool

    :param rdfs: Whether RDFS inference is also done (used in subclassed only).
    :type rdfs: bool
    """

    def __init__(self, graph, axioms, daxioms, rdfs):
        """
        @param graph: the RDF graph to be extended
        @type graph: rdflib.Graph
        @param axioms: whether (non-datatype) axiomatic triples should be added or not
        @type axioms: bool
        @param daxioms: whether datatype axiomatic triples should be added or not
        @type daxioms: bool
        @param rdfs: whether RDFS inference is also done (used in subclassed only)
        @type rdfs: boolean
        """
        Core.__init__(self, graph, axioms, daxioms, rdfs)

    def add_axioms(self):
        """
        Add axioms
        """
        for t in RDFS_Axiomatic_Triples:
            self.graph.add(t)
        for i in range(1, self.IMaxNum + 1):
            ci = RDF[("_%d" % i)]
            self.graph.add((ci, RDF.type, RDF.Property))
            self.graph.add((ci, RDFS.domain, RDFS.Resource))
            self.graph.add((ci, RDFS.range, RDFS.Resource))
            self.graph.add((ci, RDF.type, RDFS.ContainerMembershipProperty))

    def add_d_axioms(self):
        """
        This is not really complete, because it just uses the comparison possibilities that RDFLib provides.
        """
        # #1
        literals = (lt for lt in self._literals() if lt.datatype is not None)
        for lt in literals:
            self.graph.add((lt, RDF.type, lt.datatype))

        for t in RDFS_D_Axiomatic_Triples:
            self.graph.add(t)

    # noinspection PyBroadException
    def one_time_rules(self):
        """
        Some of the rules in the rule set are axiomatic in nature, meaning that they really have to be added only
        once, there is no reason to add these in a cycle. These are performed by this method that is invoked only once
        at the beginning of the process.

        In this case this is related to a 'hidden' same as rules on literals with identical values (though different
        lexical values).
        """
        # There is also a hidden sameAs rule in RDF Semantics: if a literal appears in a triple, and another one has
        # the same value, then the triple should be duplicated with the other value.
        literals = self._literals()
        items = (
            (lt1, lt2)
            for lt1, lt2 in product(literals, literals)
            if lt1.value == lt2.value
        )
        for lt1, lt2 in items:
            # In OWL, this line is simply stating a sameAs for the
            # corresponding literals, and then let the usual rules take
            # effect. In RDFS this is not possible, so the sameAs rule is,
            # essentially replicated...
            for (s, p, o) in self.graph.triples((None, None, lt1)):
                self.graph.add((s, p, lt2))

    def rules(self, t, cycle_num):
        """
        Go through the RDFS entailment rules rdf1, rdfs4-rdfs12, by extending the graph.

        :param t: A triple (in the form of a tuple).
        :type t: tuple

        :param cycle_num: Which cycle are we in, starting with 1. Can be used for some (though minor) optimization.
        :type cycle_num: int
        """
        s, p, o = t
        # rdf1
        self.store_triple((p, RDF.type, RDF.Property))
        # rdfs4a
        if cycle_num == 1:
            self.store_triple((s, RDF.type, RDFS.Resource))
        # rdfs4b
        if cycle_num == 1:
            self.store_triple((o, RDF.type, RDFS.Resource))
        if p == RDFS.domain:
            # rdfs2
            for uuu, Y, yyy in self.graph.triples((None, s, None)):
                self.store_triple((uuu, RDF.type, o))
        if p == RDFS.range:
            # rdfs3
            for uuu, Y, vvv in self.graph.triples((None, s, None)):
                self.store_triple((vvv, RDF.type, o))
        if p == RDFS.subPropertyOf:
            # rdfs5
            for Z, Y, xxx in self.graph.triples((o, RDFS.subPropertyOf, None)):
                self.store_triple((s, RDFS.subPropertyOf, xxx))
            # rdfs7
            for zzz, Z, www in self.graph.triples((None, s, None)):
                self.store_triple((zzz, o, www))
        if p == RDF.type and o == RDF.Property:
            # rdfs6
            self.store_triple((s, RDFS.subPropertyOf, s))
        if p == RDF.type and o == RDFS.Class:
            # rdfs8
            self.store_triple((s, RDFS.subClassOf, RDFS.Resource))
            # rdfs10
            self.store_triple((s, RDFS.subClassOf, s))
        if p == RDFS.subClassOf:
            # rdfs9
            for vvv, Y, Z in self.graph.triples((None, RDF.type, s)):
                self.store_triple((vvv, RDF.type, o))
            # rdfs11
            for Z, Y, xxx in self.graph.triples((o, RDFS.subClassOf, None)):
                self.store_triple((s, RDFS.subClassOf, xxx))
        if p == RDF.type and o == RDFS.ContainerMembershipProperty:
            # rdfs12
            self.store_triple((s, RDFS.subPropertyOf, RDFS.member))
        if p == RDF.type and o == RDFS.Datatype:
            self.store_triple((s, RDFS.subClassOf, RDFS.Literal))
        
        ## added rules
        if p == RDF.type:          
            for Z, Y, xxx in self.graph.triples((o, EX.has_a, None)):
                new_uri = URIRef(xxx.replace(o,s))
                self.store_triple((s, EX.has_a, new_uri))
                #print(xxx)
                xxx_type = self.query_type(xxx)
                if xxx_type:
                    self.store_triple((new_uri, RDF.type, xxx_type))
                for W, V, uuu in self.graph.triples((xxx, EX.connects, None)):
                    new_uri2 = URIRef(uuu.replace(o,s))
                    self.store_triple((new_uri, EX.connects, new_uri2))
                    #xxx_type = self.query_type(xxx)
                    #self.store_triple((new_uri, RDF.type, xxx_type))

            for Z, Y, xxx in self.graph.triples((o, EX.connects, None)):
                new_uri = URIRef(xxx.replace(o,s))
                self.store_triple((s, EX.connects, new_uri))
                xxx_type = self.query_type(xxx)
                if xxx_type:
                    self.store_triple((new_uri, RDF.type, xxx_type))

            for Z, Y, xxx in self.graph.triples((o, EX.affordance, None)):
                #new_uri = URIRef(xxx.replace(o,s))
                self.store_triple((s, EX.affordance, xxx))
                #xxx_type = self.query_type(xxx)
                #self.store_triple((new_uri, RDF.type, xxx_type))

            for Z, Y, xxx in self.graph.triples((o, RDF.type, None)):
                self.store_triple((s, RDF.type, xxx))

            # for Z, Y, xxx in self.graph.triples((o, EX.connects, None)):
            #     new_uri = URIRef(xxx.replace(o,s))
            #     self.store_triple((s, EX.connects, new_uri))

            # for Z, Y, xxx in self.graph.triples((o, EX.conforms_to, None)):
            #     self.store_triple((s, EX.conforms_to, xxx))
            
            # for Z, Y, xxx in self.graph.triples((o, EX.affordance, None)):
            #     self.store_triple((s, EX.affordance, xxx))

        


        if p == EX.connects:
            self.store_triple((o, p, s))
    
    

    #def unique_uri()
        

    def _literals(self):
        """
        Get all literals defined in the graph.
        """
        return set(o for s, p, o in self.graph if isinstance(o, Literal))

    def query_type(self, subject):
        query = """
            PREFIX ex: <http://example.com/>

            SELECT ?object
            WHERE {
                ?subject rdf:type ?object .
            }"""
        answer = []
        for r in self.graph.query(query, initBindings={'subject': URIRef(subject)}):
                answer.append(r[0])
        if answer:
            return answer[0]
        else:
            return None