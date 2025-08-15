import time
import matplotlib.pyplot as plt
from SPARQLWrapper import SPARQLWrapper, JSON
from itertools import product

sparql = SPARQLWrapper("https://knowledgedb.informatik.uni-bremen.de/mealprepDB/MealPreparation/query")
sparql.setReturnFormat(JSON)

prefix = """
PREFIX owl: <http://www.w3.org/2002/07/owl#>
prefix dul: <http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#>
PREFIX pour: <http://www.ease-crc.org/ont/meals#>
PREFIX rdf: <http://www.w3.org/1999/02/22-rdf-syntax-ns#>
PREFIX rdfs: <http://www.w3.org/2000/01/rdf-schema#>
PREFIX FOODON: <http://purl.obolibrary.org/obo/>
PREFIX soma: <http://www.ease-crc.org/ont/SOMA.owl#>
PREFIX sit_aware: <http://www.ease-crc.org/ont/situation_awareness#>
PREFIX obo: <http://purl.obolibrary.org/obo/>
PREFIX qudt: <http://qudt.org/schema/qudt#>
Prefix meals: <http://www.ease-crc.org/ont/meals#>
PREFIX ENVO: <http://purl.obolibrary.org/obo/>
PREFIX UBERON: <http://purl.obolibrary.org/obo/>
"""

food_object_names = {
    "FOODON:03310334": "yeast",
    "FOODON:03301940": "baking powder",
    "FOODON:00002385": "baking soda",
    "FOODON:00001649": "pepper",
    "FOODON:03301777": "butter",
    "FOODON:03316061": "chicken egg",
    "FOODON:03307240": "chocolate",
    "FOODON:03301304": "batter",
    "FOODON:00001260": "beer",
    "FOODON:03420170": "broth",
    "FOODON:03301564": "champagne",
    "FOODON:03301036": "coffee",
    "FOODON:03315498": "dressing",
    "ENVO:00003064": "water",
    "FOODON:00001178": "honey",
    "FOODON:03315552": "juice",
    "UBERON:0001913": "milk",
    "FOODON:03310387": "oil",
    "FOODON:03311146": "sauce",
    "FOODON:00003280": "soup",
    "FOODON:03303225": "syrup",
    "FOODON:03315081": "tea",
    "FOODON:03301705": "vinegar",
    "FOODON:03301338": "wine",
    "FOODON:03306347": "pasta",
    "FOODON:00004327": "rice",
    "FOODON:03301073": "sugar",
    "FOODON:03309954": "salt",
    "FOODON:03302339": "flour",
}

def get_needed_tool(verb):
    query = """
     SELECT ?res WHERE {
                        %s rdfs:subClassOf ?sub.
                        ?sub owl:onProperty dul:hasParticipant.
                        ?sub owl:someValuesFrom ?neededtool.
                        BIND(REPLACE(STR(?neededtool), "^.*[#/]", "") AS ?res).
                    }
    """ % (verb)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else None

def get_min_angle(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringAngle.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:minQualifiedCardinality ?res.
                    }
                    """ % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "0"

def get_max_angle(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringAngle.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:maxQualifiedCardinality ?res.
                    }""" % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "90"

def get_min_duration(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringDuration.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:minQualifiedCardinality ?res.
                    }
                    """ % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "0"

def get_max_duration(foodobject):
    query = """
    SELECT ?res WHERE {
                        %s rdfs:subClassOf ?cons_node.
                        ?cons_node owl:onProperty pour:hasConsistency.
			?cons_node owl:someValuesFrom ?consistency.
                        ?union rdf:first*/rdf:rest* ?consistency.
 			 {
 			 ?rest rdf:first*/rdf:rest* ?union.
  			?start owl:unionOf ?rest.
 			 ?a owl:someValuesFrom ?start.
 			 }
 			 UNION
 			 {
 			   ?a owl:allValuesFrom ?union.
 			 }
 			 ?f rdf:first*/rdf:rest* ?a.
			  ?r rdf:first*/rdf:rest* ?f.
			  ?i owl:intersectionOf ?r.
			  ?inter owl:someValuesFrom ?i.
			  ?param rdfs:subClassOf ?inter.
			  ?param rdfs:subClassOf pour:PouringDuration.
			  ?param rdfs:subClassOf ?degree1.
			  ?degree1 owl:onProperty qudt:valueQuantity.
			  ?degree1 owl:maxQualifiedCardinality ?res.
                    }
    """ % (foodobject)
    full_query = (prefix + query)
    sparql.setQuery(full_query)
    results = sparql.queryAndConvert()
    return results["results"]["bindings"][0]["res"]["value"] if results["results"]["bindings"] else "10"


def query_var(verb, foodobject):
    print(f"For the verb {verb} and food object {foodobject}, the needed tool is:{get_needed_tool(verb)}")
    print(f"For the verb {verb} and food object {foodobject}, the minimum angle is:{get_min_angle(foodobject)}")
    print(f"For the verb {verb} and food object {foodobject}, the maximum angle is:{get_max_angle(foodobject)}")
    print(f"For the verb {verb} and food object {foodobject}, the minimum duration is:{get_min_duration(foodobject)}")
    print(f"For the verb {verb} and food object {foodobject}, the maximum duration is:{get_max_duration(foodobject)}")

def run_query(query):
    try:
        sparql.setQuery(prefix + query)
        results = sparql.queryAndConvert()
        bindings = results.get("results", {}).get("bindings", [])
        if bindings:
            return bindings[0]["res"]["value"]
        return None
    except Exception as e:
        print(f"Query failed: {e}")
        return None

# Include your query functions here (get_needed_tool, get_min_angle, etc.)
# [Paste your previous query functions here]

def test_permutations(verbs, food_objects, runs_per_pair=3):
    results = []

    for verb, foodobject in product(verbs, food_objects):
        food_name = food_object_names.get(foodobject, foodobject)  # Make sure you have this dictionary defined
        for run_i in range(runs_per_pair):
            start = time.time()
            try:
                tool = get_needed_tool(verb)
                min_angle = get_min_angle(foodobject)
                max_angle = get_max_angle(foodobject)
                min_dur = get_min_duration(foodobject)
                max_dur = get_max_duration(foodobject)
                duration = time.time() - start

                success = any(param is not None for param in [tool, min_angle, max_angle, min_dur, max_dur])

                results.append({
                    'verb': verb,
                    'foodobject': foodobject,
                    'food_name': food_name,
                    'run': run_i + 1,
                    'duration': duration,
                    'success': success
                })

                status = 'Success' if success else 'Fail'
                print(f"Verb={verb}, Food={food_name}, Run={run_i+1}, Time={duration:.3f}s, {status}")

            except Exception as e:
                print(f"Error on Verb={verb}, Food={food_name}, Run={run_i+1}: {e}")
                results.append({
                    'verb': verb,
                    'foodobject': foodobject,
                    'food_name': food_name,  # Add here too
                    'run': run_i + 1,
                    'duration': None,
                    'success': False
                })

    return results

def plot_results(results):
    import pandas as pd
    import matplotlib.pyplot as plt

    df = pd.DataFrame(results)

    summary = df.groupby(['verb', 'foodobject', 'food_name']).agg({
        'duration': 'mean'
    }).reset_index()

    plt.figure(figsize=(16, 7))

    colors = plt.cm.get_cmap('tab10')

    food_names = summary['food_name'].unique()
    x = range(len(food_names))

    for i, verb in enumerate(summary['verb'].unique()):
        subset = summary[summary['verb'] == verb]
        y_duration = [subset[subset['food_name'] == fn]['duration'].values[0] if fn in subset['food_name'].values else None for fn in food_names]

        plt.plot(x, y_duration, marker='o', linestyle='-', label=verb, color=colors(i))

    # Increase font size for ticks, labels, title and legend
    plt.xticks(x, food_names, rotation=45, ha='right', fontsize=16)
    plt.yticks(fontsize=16)
    plt.ylabel('Average Query Duration (s)', fontsize=18)
    plt.title('Average Query Duration per Food Object', fontsize=20)
    plt.legend(loc='upper right', fontsize=16)
    plt.tight_layout()
    plt.savefig('avg_query_duration_per_food.png')
    plt.show()



if __name__ == "__main__":
    verbs = [
        "meals:Draining",
        "meals:Pouring",
        "meals:Cascading",
        "meals:Flowing",
        "meals:Splashing",
        "meals:Streaming",
        "meals:PouringThrough",
        "meals:Spilling",
        "meals:Sprinkling",
        # "meals:Crumbling"
    ]
    food_objects = [
        "FOODON:03310334",  # yeast
        "FOODON:03301940",  # baking powder
        "FOODON:00002385",  # baking soda
        "FOODON:00001649",  # pepper
        "FOODON:03301777",  # butter
        "FOODON:03316061",  # chicken egg
        "FOODON:03307240",  # chocolate
        "FOODON:03301304",  # batter
        "FOODON:00001260",  # beer
        "FOODON:03420170",  # broth
        "FOODON:03301564",  # champagne
        "FOODON:03301036",  # coffee
        "FOODON:03315498",  # dressing
        "ENVO:00003064",  # water
        "FOODON:00001178",  # honey
        "FOODON:03315552",  # juice
        "UBERON:0001913",  # milk
        "FOODON:03310387",  # oil
        "FOODON:03311146",  # sauce
        "FOODON:00003280",  # soup
        "FOODON:03303225",  # syrup
        "FOODON:03315081",  # tea
        "FOODON:03301705",  # vinegar
        "FOODON:03301338",  # wine
        "FOODON:03306347",  # pasta
        "FOODON:00004327",  # rice
        "FOODON:03301073",  # sugar
        "FOODON:03309954",  # salt
        "FOODON:03302339"  # flour
    ]

    # Paste your query functions here before running the test

    results = test_permutations(verbs, food_objects, runs_per_pair=3)
    plot_results(results)
