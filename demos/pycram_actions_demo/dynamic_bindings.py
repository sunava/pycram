from doc.source.notebooks.tmp.orm_example import milk_desig
from pycram.helper import perform
from pycram.language import SequentialPlan as seq
from pycram.parameterizer import Parameterizer as QueryInterface
from pycram.process_module import simulated_robot
from pycram.robot_plans import NavigateAction, PouringAction, PickUpAction


def ACTIONDESIGANTORPICKUP(desig):
    query_interface = QueryInterface(desig)

    var_bindings = query_interface.query_var(pick_up_variable, ..)



    for i in range(10):
        sample = var_bindings.sample(1)

        resolved_plan = query_interface.plan_from_sample(var_bindings, sample[0])
        return resolved_plan



desig =
resolved_plan = resolve(desig)
perform(resolved_plan)

# fridge location
# pre_task
# michael will hier dei frage die man in die query var reinsteckt sehen
# perform in simualtion mit var


# #kann man das als frage fuer die zukunft ausdruecken -> wird dieser designator in der simulation erfolgreich sein?
#
# wenn in mental succesfull dann mach weiter
# detect milk_desig
# designator belioef des roboters die aktion erfolreich ausgefuellt werden erfolgreiches modell
# mit with policies, so lange du glaubst dass du die aktion erfolgreich ausfuehren kannst, fuehre sie aus
# sobald du glaubst dass du die aktion nicht erfolgreich ausfuehren kannst, fuehre sie nicht aus und generier nen fehler
# und werf die mit ner fehler bechreibung nach oben
# execution failure monitors
# ich guck immer ob der roboter glaubt dassdie aktion erfolgreich zuende gefuehrt werden kann und sobald das spring
# ueber der basis von probabilistischen distribution wird der erfolgreich sein? wahrscheinlichkeitverteilung und angeben
# wie hoch ist die wahrscheinlichkeit dass ich die aktion erfolgreich ausfuehren kann?


# wo ist milch
# wie stehen fuer grasp





#
#
#
# resolve(Action)
#
#
#
#
# resolve(x,y,z(an(Action(type=PickUp, =target milk))
#
#     fridge_location= ...
#   query _var ....=
#
#
#
#
#
#
#
#
#
#  )
#
#  def resolve2(*args, **kwargs):
#      ...
#
#  resolve2(Action, fridge_location=..., query_var=..., target=milk_desig, functional_programming=True, with_policz_and_log_it_all_into_knowrob_and_cashapp_me_500_euros=True)