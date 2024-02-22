from pycram.designators.location_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose

breakfast_objects = {"milk": {"type": "milk", "model": "milk.stl", "pose": [2.5, 2, 1.02],
                              "color": [1, 0, 0, 1], "default_location": "island_countertop"},
                     "breakfast_cereal": {"type": "breakfast_cereal", "model": "breakfast_cereal.stl",
                                "pose": [2.5, 2.5, 1.05],
                                "color": [0, 1, 0, 1], "default_location": "island_countertop"},
                     "spoon": {"type": "spoon", "model": "spoon.stl", "pose": [2.4, 2.2, 0.85],
                               "color": [0, 0, 1, 1], "default_location": "cabinet10_drawer_top"},
                     "bowl": {"type": "bowl", "model": "bowl.stl", "pose": [2.38, 2.2, 1.02],
                              "color": [1, 1, 0, 1], "default_location": "island_countertop"}}
test_spoon = {"spoon": {"type": "spoon", "model": "spoon.stl", "pose": [2.43, 2.2, 0.85],
                        "color": [0, 0, 1, 1], "default_location": "drawer"}}


class ContextConfig:
    environment_object = None
    spoon_ = None

    def __init__(self, context_name, enviornment_name, objects_info):
        self.context_name = context_name
        self.objects_info = objects_info  # A dictionary of object names and their types
        self.environment_name = enviornment_name  # Name of the environment model

    def spawn_objects(self):
        if self.environment_name == "apartment":
            path = "apartment-small.urdf"
        elif self.environment_name == "kitchen":
            path = "kitchen-small.urdf"
        apart = Object("apartment", ObjectType.ENVIRONMENT, path)

        # Function to spawn objects in the simulation environment
        for obj_name, obj_info in self.objects_info.items():
            obj = Object(obj_name, obj_info['type'], obj_info['model'], pose=Pose(obj_info['pose']),
                         color=obj_info['color'])
            if obj_info['type'] == "spoon":
                apart.attach(obj, 'cabinet10_drawer_top')
                self.spoon_ = obj
        self.environment_object = apart

    def search_locations(self, object_name):
        # Function to provide likely locations of objects within this context
        if object_name in self.objects_info:
            return self.objects_info[object_name].get('default_location', 'Unknown')
        return 'Unknown'

    def get_all_objects(self):
        return self.objects_info.keys()


breakfast_context_apartment = ContextConfig("breakfast", "apartment", breakfast_objects)
test_context_apartment = ContextConfig("test_spoon", "apartment", test_spoon)
