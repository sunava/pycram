import rospkg

from rdflib import Graph, Literal, URIRef, Namespace
from rdflib.namespace import OWL, RDF, RDFS


class SPARQL():
    def __init__(self):

        rospack = rospkg.RosPack()
        self.query_folder: str = rospack.get_path('pycram') + '/demos/pycram_food_cutting_demo/queries'

        self.knowledge_graph = Graph()

        # define prefixes to be used in the query
        SOMA = Namespace("http://www.ease-crc.org/ont/SOMA.owl#")
        CUT2 = Namespace("http://www.ease-crc.org/ont/situation_awareness#")
        CUT = Namespace("http://www.ease-crc.org/ont/food_cutting#")
        DUL = Namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")
        OBO = Namespace("http://purl.obolibrary.org/obo/")
        self.knowledge_graph.bind("owl", OWL)
        self.knowledge_graph.bind("rdfs", RDFS)
        self.knowledge_graph.bind("soma", SOMA)
        self.knowledge_graph.bind("cut2", CUT2)
        self.knowledge_graph.bind("cut", CUT)
        self.knowledge_graph.bind("dul", DUL)
        self.knowledge_graph.bind("obo", OBO)

    def repetitions(self, task, onotlogy_obj):
        task = task
        tobject = onotlogy_obj
        repetitionsquery = """  SELECT ?rep WHERE {
              SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {  
          {
             OPTIONAL{ %s rdfs:subClassOf ?action}
                ?action rdfs:subClassOf* ?rep_node.
                ?rep_node owl:onProperty cut:repetitions.
                FILTER EXISTS {
                    ?rep_node owl:hasValue ?val.}
                BIND("1" AS ?rep)}
            UNION
            {
               OPTIONAL{ %s rdfs:subClassOf ?action }
                ?action rdfs:subClassOf* ?rep_node.
                ?rep_node owl:onProperty cut:repetitions.
                FILTER EXISTS {
                    ?rep_node owl:minQualifiedCardinality ?val.}
                BIND("more than 1" AS ?rep)}} }""" % (task, task)
        for row in self.knowledge_graph.query(repetitionsquery):
            return row.rep

    #if halving then middle of object if sclicing end or beginning of object and + repitions is then technique
    def position(self, task, onotlogy_obj):
        task = task
        tobject = onotlogy_obj
        positionquery = """  SELECT ?position WHERE {
              SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {  
          OPTIONAL { %s rdfs:subClassOf ?sub.}
          ?sub rdfs:subClassOf* ?node.
          ?node owl:onProperty cut:affordsPosition.
          ?node owl:someValuesFrom ?position.
        } }""" % (task)

        for row in self.knowledge_graph.query(positionquery):
            return row.position, row.position.split("#", 1)[1]

    def prior_task(self, task, onotlogy_obj):
        task = task
        tobject = onotlogy_obj
        prioractionquery = """  SELECT ?priortask WHERE {
              SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {  
          %s rdfs:subClassOf ?sub.
          ?sub owl:onProperty cut:requiresPriorTask .
          ?sub owl:someValuesFrom ?priortask.
        } }""" % (task)

        for row in self.knowledge_graph.query(prioractionquery):
            return row.priortask,  row.priortask.split("#", 1)[1]

    def get_cutting_tool(self, task, onotlogy_obj):
        query = """SELECT ?alltools WHERE {
                  SERVICE <https://api.krr.triply.cc/datasets/mkumpel/FruitCuttingKG/services/FoodCuttingKG/sparql> {
                  %s rdfs:subClassOf ?node.
                  ?node owl:onProperty soma:hasDisposition.
                  ?node owl:someValuesFrom ?collection.
                  ?collection owl:intersectionOf ?node2.
                  ?node2 rdf:first cut2:Cuttability.
                  ?node2 rdf:rest ?toolnode.
                  ?toolnode rdf:rest ?collection2.
                  ?collection2 rdf:first ?tooluse.
                  ?tooluse owl:onProperty soma:affordsTrigger.
                  ?tooluse owl:allValuesFrom ?tool.
                  ?tool owl:onProperty dul:classifies.
                  ?tool owl:allValuesFrom ?whattool.
                  ?alltools rdfs:subClassOf* ?whattool.} }""" % (onotlogy_obj)
        for row in self.knowledge_graph.query(query):
            return row.alltools, row.alltools.split("#", 1)[1]


