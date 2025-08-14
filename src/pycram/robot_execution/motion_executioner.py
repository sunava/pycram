from pycram.external_interfaces import giskard
from pycram.plan import Plan, MotionNode
from pycram.robot_description import RobotDescription
from pycram.robot_plans import BaseMotion

# --- generischer executor ---
def execute_with_giskard(motion: BaseMotion):
    func_name = f"map_{type(motion).__name__}"
    mapper = globals().get(func_name)
    if not callable(mapper):
        raise ValueError(f"No Giskard mapper function found for {type(motion).__name__}")
    mapper(motion)

def execute_leaf_motions_with_giskard(plan: Plan):
    for node in plan.nodes:
        if isinstance(node, MotionNode) and not node.children:
            print("Executing:", node.designator_ref)
            execute_with_giskard(node.designator_ref)
