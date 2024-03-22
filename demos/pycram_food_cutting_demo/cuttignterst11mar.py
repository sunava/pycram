
from pycram.resolver.action.SPARQL import SPARQL


query_resolver = SPARQL()
repititions_ont = query_resolver.repetitions(task="soma:Dicing", onotlogy_obj="obo:FOODON_00003415")
print(repititions_ont)

position_ont, position_name = query_resolver.position(task="soma:Dicing", onotlogy_obj="obo:FOODON_00003415")
print(position_ont, position_name)
prio_task_ont, prio_task = query_resolver.prior_task(task="soma:Dicing", onotlogy_obj="obo:FOODON_00003415")
print(prio_task_ont, prio_task)
cutting_tool_ont, cutting_tool = query_resolver.get_cutting_tool(task="soma:Dicing", onotlogy_obj="obo:FOODON_00003415")
print(cutting_tool_ont, cutting_tool)


