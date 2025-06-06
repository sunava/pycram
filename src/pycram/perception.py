from .designators.action_designator import ActionDescription
from .designators.motion_designator import *
from .world_concepts.world_object import Object
from .object_descriptors.generic import ObjectDescription as GenericObjectDescription
from pycrap.urdf_parser import parse_furniture
from .external_interfaces.robokudo import *
from .ros import get_time
from .datastructures.dataclasses import Colors
from typing_extensions import List, TYPE_CHECKING
from .designators.object_designator import *

if TYPE_CHECKING:
    from .designators.object_designator import ObjectDesignatorDescription

extension = ObjectDescription.get_file_extension()


def detect(believe_object: BelieveObject) -> Any:
    """
     Perform a perception query and inject the perceived object(s) back into the BelieveObject.
     Example:
         BelieveObject(types=["Bowl"])
         BelieveObject(names=["red_cup"])
     """
    if hasattr(believe_object, "types") and believe_object.types:
        searched_obj = believe_object.types
        print(f"→ Querying by types: {believe_object.types}")

    else:
        searched_obj = believe_object.names
        print(f"→ Querying by names: {believe_object.names}")

    query_result = query_all_objects()

    if query_result is None or []:
        raise PerceptionObjectNotFound("Could not find an object of type(s): " + str(believe_object.types))
    perceived_objects = parse_query_result(query_result, searched_obj)
    if perceived_objects is None or []:
        raise PerceptionObjectNotFound("No matching objects found for types: " + str(believe_object.types))

    return perceived_objects


#
# def should_query_object(key: str, value) -> bool:
#     return (
#             key == "_object"
#             or key.endswith("_object")
#             or key.endswith("_objects")
#             or isinstance(value, Object)
#     )


def parse_query_result(query_result, searched_type: List[str]) -> List[Object]:
    objects = []
    print(searched_type)
    print(query_result)
    for result in query_result.res:
        try:
            obj_pose = extract_pose(result)
        except ValueError as e:
            loginfo(f"Skipping result due to invalid pose: {e}")
            continue
        obj_type = result.type
        half_size = extract_size(result)
        color = extract_color(result)
        concept = parse_furniture(obj_type) or fallback_type(obj_type)
        if concept not in searched_type:
            loginfo(f"Type '{concept}' not in searched types {searched_type}. Skipping.")
            continue

        name = f"{obj_type}_{get_time()}"
        if concept:
            concept_name = str(concept.name).lower()
            print(concept_name)
            path = f"{concept_name}.stl"
            description = None
        else:
            path = None
            description = GenericObjectDescription(name, [0, 0, 0], half_size)
            print("spwaning primitives")

        obj = Object(
            name=name,
            concept=concept,
            path=path,
            description=description,
            color=color,
            size=half_size
        )

        obj.set_pose(obj_pose)
        objects.append(obj)

    return objects


def extract_pose(result) -> PoseStamped:
    print(result)
    pose_msg_list = result.pose

    if isinstance(pose_msg_list, list):
        if len(pose_msg_list) == 0:
            raise ValueError("Empty pose list in result")
        pose_msg = pose_msg_list[0]
    else:
        pose_msg = pose_msg_list

    pycram_pose = PoseStamped.from_ros_message(pose_msg)
    return pycram_pose


def extract_size(result) -> List[float]:
    try:
        dims = result.shape_size[0].dimensions
        return [dims.x / 2, dims.y / 2, dims.z / 2]
    except IndexError:
        return [0.2, 0.2, 0.2]


def extract_color(result):
    try:
        return Colors.from_string(result.color[0])
    except IndexError:
        return Colors.PINK


def fallback_type(type_str: str):
    loginfo(f"No class name contains the string '{type_str}'")
    return PhysicalObject

#
#
# generic_obj_BO = BelieveObject(names=["jeroen_cup"])
# tool_BO = BelieveObject(names=["bowl"])
#
#
# # Example grounded object
# class GroundedObject:
#     def __init__(self, name, pose):
#         self.name = name
#         self.pose = pose
#
#     def __repr__(self):
#         return f"GroundedObject(name={self.name}, pose={self.pose})"
#
#
# def detect_objects_from_designator(designator):
#     name = designator.names[0]
#     print(f"[Perception] Detecting object with name: {name}")
#     # Simulated result
#     return [GroundedObject(name=name, pose="pose_of_" + name)]
#
#
# def detect(action_or_expr):
#     def resolve_designators(obj):
#         if isinstance(obj, BelieveObject) and obj.grounded_object is None:
#             result = detect_objects_from_designator(obj)
#             if result:
#                 obj.grounded_object = result[0]
#         elif isinstance(obj, list):
#             for item in obj:
#                 resolve_designators(item)
#         elif isinstance(obj, dict):
#             for v in obj.values():
#                 resolve_designators(v)
#         elif hasattr(obj, '__dict__'):
#             for attr in obj.__dict__.values():
#                 resolve_designators(attr)
#
#     resolve_designators(action_or_expr)
#     return action_or_expr
