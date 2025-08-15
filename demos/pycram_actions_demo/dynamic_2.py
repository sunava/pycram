from __future__ import annotations

from dataclasses import dataclass, field
from datetime import timedelta

import numpy as np
from typing_extensions import Union, Optional, Type, Any, Iterable

from pycram.datastructures.dataclasses import FrozenObject
from pycram.datastructures.enums import Arms
from pycram.datastructures.partial_designator import PartialDesignator
from pycram.datastructures.pose import PoseStamped
from pycram.designators.location_designator import ProbabilisticCostmapLocation
from pycram.designators.object_designator import BelieveObject
from pycram.failures import ObjectUnfetchable, ReachabilityFailure
from pycram.has_parameters import has_parameters
from pycram.plan import with_plan
from pycram.robot_description import RobotDescription
from pycram.robot_plans import ActionDescription, record_object_pre_perform, ParkArmsActionDescription, \
    NavigateActionDescription, PickUpActionDescription
from pycram.world_concepts.world_object import Object


def with_policy(perform_func):
    def wrapper(self, *args, **kwargs):
        # Call validate if it exists
        valid = True
        if hasattr(self, "validate") and callable(self.validate):
            valid = self.validate()
        if valid:
            return perform_func(self, *args, **kwargs)
        else:
            print(f"Policy prevented performing {self.__class__.__name__}")
            # Optionally raise or silently skip
    return wrapper


class ActionContext:
    def __init__(self, action):
        self.action = action
    def get_params(self, keys):
        params = {}
        for key in keys:
            # Try to get parameter value using the class's _parameters dict if it exists
            if hasattr(self.action.__class__, "_parameters") and key in self.action.__class__._parameters:
                # Assuming each parameter is stored as a property or field
                val = getattr(self.action, key, None)
            else:
                val = None
            if val is not None:
                params[key] = val
        return params



def query_var(action_type, var_name, query_type, *, context=None, **explicit_params):
    print(f"[Query] {action_type}.{var_name} via {query_type}")
    robot_desig_resolved = BelieveObject(names=[RobotDescription.current_robot_description.name]).resolve()
    # Define keys needed for costmap queries
    costmap_keys = [
        "object_designator",
        "reachable_for",
        "reachable_arm",
        "grasp_descriptions",
        "object_in_hand",
        "rotation_agnostic",
    ]

    if context and query_type == "costmap":
        # Fill missing keys from context
        ctx_params = context.get_params(costmap_keys)
        print(ctx_params)
        # Explicit params override context params
        query_params = {**ctx_params, **explicit_params}
        print(query_params)
        loc = (
            ProbabilisticCostmapLocation(**query_params, reachable_for=robot_desig_resolved))
        ti = loc
        result = loc.resolve()
        print(f"[Debug] Resolved object: {result}, type: {type(result)}")
        print(f"[Debug] Has 'arm'? {hasattr(result, 'arm')}")
        print(f"[Debug] Has 'grasp_description'? {hasattr(result, 'grasp_description')}")
        return result

    # Add other query types here...

    raise ValueError(f"Unknown query type {query_type}")

@has_parameters
@dataclass
class TransportAction1(ActionDescription):
    """
    Transports an object to a position using an arm
    """

    object_designator: Object = field(repr=False)
    """
    Object designator_description describing the object that should be transported.
    """
    target_location: PoseStamped
    """
    Target Location to which the object should be transported
    """
    arm: Optional[Arms]
    """
    Arm that should be used
    """
    place_rotation_agnostic: Optional[bool] = False
    """
    If True, the robot will place the object in the same orientation as it is itself, no matter how the object was grasped.
    """

    object_at_execution: Optional[FrozenObject] = field(init=False, repr=False, default=None)
    """
    The object at the time this Action got created. It is used to be a static, information holding entity. It is
    not updated when the BulletWorld object is changed.
    """

    _pre_perform_callbacks = []
    """
    List to save the callbacks which should be called before performing the action.
    """


    def __post_init__(self):
        super().__post_init__()

        # Store the object's data copy at execution
        self.pre_perform(record_object_pre_perform)

    def plan(self):
        context = ActionContext(self)

        ParkArmsActionDescription(Arms.BOTH).perform()
        self.target_location = XXXX
        #add object designator
        pickup_pose = query_var("TransportAction", object_designator="", var="pickup_pose", query_type="costmap", context=context)
        if not pickup_pose:
            raise ObjectUnfetchable("...")
        print("with policy pickup_pose:")
        with_policy(NavigateActionDescription(pickup_pose, True).perform())
        print("with policy pickup_pose:")
        @with_policy
        NavigateActionDescription(pickup_pose, True).perform()

        PickUpActionDescription(
            object_designator=self.object_designator,
            arm=pickup_pose.arm,
            grasp_description=pickup_pose.grasp_description
        ).perform()
        ParkArmsActionDescription(Arms.BOTH).perform()

        # place_loc = query_var("TransportAction", "place_location", "costmap", context=context)
        # if not place_loc:
        #     raise ReachabilityFailure("...")
        #
        # NavigateActionDescription(place_loc, True).perform()


    def validate(self, result: Optional[Any] = None, max_wait_time: Optional[timedelta] = None):

        pass


    @classmethod
    @with_plan
    def description(cls, object_designator: Union[Iterable[Object], Object],
                    target_location: Union[Iterable[PoseStamped], PoseStamped],
                    arm: Union[Iterable[Arms], Arms] = None, place_rotation_agnostic: Optional[bool] = False) -> \
    PartialDesignator[Type[TransportAction1]]:
        return PartialDesignator(TransportAction1, object_designator=object_designator,
                                 target_location=target_location,
                                 arm=arm, place_rotation_agnostic=place_rotation_agnostic)



TransportActionDescription1 = TransportAction1.description
