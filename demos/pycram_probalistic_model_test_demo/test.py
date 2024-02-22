from probabilistic_model.probabilistic_circuit.distributions import SymbolicDistribution
from probabilistic_model.probabilistic_circuit.probabilistic_circuit import DecomposableProductUnit, ProbabilisticCircuit
from random_events.variables import Symbolic
import json
from random_events.events import Event

with open("../../resources/models/defenitly_not_hard_coded.pm", "r") as f:
    model = ProbabilisticCircuit.from_json(json.load(f))

mode , likelihood = model.mode()

location, pyobject, tool = model.variables
#print(model.mode())
event = Event({location: "drawer", tool: "knife"})
con, proba = model.conditional(event)

mode, likelihood = con.mode()

print(mode)

#print(mode[0]["object"])