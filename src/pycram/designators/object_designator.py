from __future__ import annotations
import dataclasses
from typing_extensions import List, Optional, Callable, TYPE_CHECKING
import sqlalchemy.orm
from ..designator import ObjectDesignatorDescription
from ..orm.base import ProcessMetaData
from ..orm.object_designator import (BelieveObject as ORMBelieveObject, ObjectPart as ORMObjectPart)
from ..datastructures.pose import Pose
from std_msgs.msg import String
from pycram.worlds.bullet_world import BulletWorld, Object as BulletWorldObject
from ..external_interfaces.robokudo import query

if TYPE_CHECKING:
    import owlready2

class BelieveObject(ObjectDesignatorDescription):
    """
    Description for Objects that are only believed in.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        """
        Concrete object that is believed in.
        """

        def to_sql(self) -> ORMBelieveObject:
            return ORMBelieveObject(self.obj_type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMBelieveObject:
            metadata = ProcessMetaData().insert(session)
            self_ = self.to_sql()
            self_.process_metadata = metadata
            session.add(self_)

            return self_


class ObjectPart(ObjectDesignatorDescription):
    """
    Object Designator Descriptions for Objects that are part of some other object.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):

        # The rest of attributes is inherited
        part_pose: Pose

        def to_sql(self) -> ORMObjectPart:
            return ORMObjectPart(self.obj_type, self.name)

        def insert(self, session: sqlalchemy.orm.session.Session) -> ORMObjectPart:
            metadata = ProcessMetaData().insert(session)
            pose = self.part_pose.insert(session)
            obj = self.to_sql()
            obj.process_metadata = metadata
            obj.pose = pose
            session.add(obj)

            return obj

    def __init__(self, names: List[str],
                 part_of: ObjectDesignatorDescription.Object,
                 type: Optional[str] = None,
                 resolver: Optional[Callable] = None):
        """
        Describing the relationship between an object and a specific part of it.

        :param names: Possible names for the part
        :param part_of: Parent object of which the part should be described
        :param type: Type of the part
        :param resolver: An alternative specialized_designators to resolve the input parameter to an object designator
        :param ontology_concept_holders: A list of ontology concepts that the object part is categorized as or associated with
        """
        super().__init__(names, type, resolver)

        if not part_of:
            raise AttributeError("part_of cannot be None.")

        self.type: Optional[str] = type
        self.names: Optional[List[str]] = names
        self.part_of = part_of

    def ground(self) -> Object:
        """
        Default specialized_designators, returns the first result of the iterator of this instance.

        :return: A resolved object designator
        """
        return next(iter(self))

    def __iter__(self):
        """
        Iterates through every possible solution for the given input parameter.

        :yield: A resolved Object designator
        """
        for name in self.names:
            if name in self.part_of.world_object.link_name_to_id.keys():
                yield self.Object(name, self.type, self.part_of.world_object,
                                  self.part_of.world_object.get_link_pose(name))


class LocatedObject(ObjectDesignatorDescription):
    """
    Description for KnowRob located objects.
    **Currently has no specialized_designators**
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        reference_frame: str
        """
        Reference frame in which the position is given
        """
        timestamp: float
        """
        Timestamp at which the position was valid
        """

    def __init__(self, names: List[str], types: List[str],
                 reference_frames: List[str], timestamps: List[float], resolver: Optional[Callable] = None,
                 ontology_concept_holders: Optional[List[owlready2.Thing]] = None):
        """
        Describing an object resolved through knowrob.

        :param names: List of possible names describing the object
        :param types: List of possible types describing the object
        :param reference_frames: Frame of reference in which the object position should be
        :param timestamps: Timestamps for which positions should be returned
        :param resolver: An alternative specialized_designators that resolves the input parameter to an object designator.
        :param ontology_concept_holders: A list of ontology concepts that the object is categorized as
        """
        super(LocatedObject, self).__init__(names, types, resolver, ontology_concept_holders)
        self.reference_frames: List[str] = reference_frames
        self.timestamps: List[float] = timestamps


class RealObject(ObjectDesignatorDescription):
    """
    Object designator representing an object in the real world, when resolving this object designator description ]
    RoboKudo is queried to perceive an object fitting the given criteria. Afterward the resolver tries to match
    the found object to an Object in the BulletWorld.
    """

    @dataclasses.dataclass
    class Object(ObjectDesignatorDescription.Object):
        pose: Pose
        """
        Pose of the perceived object
        """

    def __init__(self, names: Optional[List[str]] = None, types: Optional[List[str]] = None,
                 bullet_world_object: BulletWorldObject = None, resolver: Optional[Callable] = None):
        """

        :param names:
        :param types:
        :param bullet_world_object:
        :param resolver:
        """
        super().__init__(resolver)
        self.types: Optional[List[str]] = types
        self.names: Optional[List[str]] = names
        self.bullet_world_object: BulletWorldObject = bullet_world_object

    def __iter__(self):
        """
        Queries RoboKudo for objects that fit the description and then iterates over all BulletWorld objects that have
        the same type to match a BulletWorld object to the real object.

        :yield: A resolved object designator with reference bullet world object
        """
        object_candidates = query(self)
        for obj_desig in object_candidates:
            for bullet_obj in BulletWorld.get_objects_by_type(obj_desig.type):
                obj_desig.bullet_world_object = bullet_obj
                yield obj_desig


class HumanDescription:
    """
    Class that represents humans. this class does not spawn a human in a simulation.
    """

    def __init__(self, name: String, fav_drink: Optional = None,
                 pose: Optional = None, attributes: Optional = None):
        """
        :param name: name of human
        :param fav_drink: favorite drink of human
        :param pose: last known pose of human
        """

        # TODO: coordinate with Perception on what is easy to implement
        # characteristics to consider: height, hair color, and age.
        # self.human_pose = Fluent()
        self.name = name
        self.fav_drink = fav_drink
        self.pose = pose
        self.attributes = attributes
        self.id = -1

        # self.human_pose_sub = rospy.Subscriber("/human_pose", PoseStamped, self.human_pose_cb)

    # def human_pose_cb(self, HumanPoseMsg):
    # """
    # callback function for human_pose Subscriber.
    # sets the attribute human_pose when someone (e.g. Perception/Robokudo) publishes on the topic
    # :param HumanPoseMsg: received message
    # """

    # self.human_pose.set_value(True)
    # rospy.loginfo("done cb")
    # rospy.sleep(10)

    def set_id(self, new_id: int):
        """
        function for changing id of human
        is given by perception with face recognition
        :param new_id: new id of human
        """
        self.id = new_id

    def set_name(self, new_name):
        """
        function for changing name of human
        :param new_name: new name of human
        """
        self.name = new_name

    def set_drink(self, new_drink):
        """
        function for changing/setting favorite drink of human
        :param new_drink: name of drink
        """
        self.fav_drink = new_drink

    def set_pose(self, new_pose):
        """
        function for changing pose of human
        :param new_pose: new pose of human
        """
        print("in set pose")
        self.pose = new_pose

    def set_attributes(self, attribute_list):
        """
        function for setting attributes
        :param attribute_list: list with attributes: gender, headgear, kind of clothes, bright/dark clothes
        """
        self.attributes = attribute_list[1]


class ShelfCompartmentDescription:
    """
    Class that represents a Compartment in a shelf but in cool and convenient
    """

    def __init__(self, height: float, placing_areas: List[List[float]], category=None):
        """
        :param height: height of compartment
        :param placing_areas: x/y-coordinate of possible placing range [x min, x max]
        :param category: category of object already standing in the compartment
        """

        if category is None:
            category = []
        else:
            self.category = category

        self.height = height
        self.placing_areas = placing_areas
        self.category = category
        # list that tracks if area x is occupied
        # we assume that the compartment is empty, therefore everything is set to False
        self.area_free = []
        for i in range(len(placing_areas)):
            self.area_free.append(False)

    def set_area_occupied(self, area: int, occ: bool):
        self.area_free[area] = occ

    def get_area_occupied(self, area: int):
        return self.area_free[area]

    def get_free_area(self):
        for area in range(len(self.placing_areas)):
            if not self.area_free[area]:
                # return arithmetic mean of area
                return (self.placing_areas[area][0] + self.placing_areas[area][1]) / 2
        return -1

    def get_placing_pose(self, obj_category: str):
        for cat in self.category:
            if cat == obj_category:
                placing_pose = self.get_free_area()
                if placing_pose != -1:
                    return placing_pose
                else:
                    return -1
        return -1
