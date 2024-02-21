from pycram.designators.location_designator import *
from pycram.enums import ObjectType
from pycram.pose import Pose

breakfast_objects = {"milk": {"type": ObjectType.MILK, "model": "milk.stl", "pose": [2.5, 2, 1.02],
                              "color": [1, 0, 0, 1], "default_location": "island_countertop"},
                     "cereal": {"type": ObjectType.BREAKFAST_CEREAL, "model": "breakfast_cereal.stl",
                                "pose": [2.5, 2.3, 1.05],
                                "color": [0, 1, 0, 1], "default_location": "island_countertop"},
                     # "spoon": {"type": ObjectType.SPOON, "model": "spoon.stl", "pose": [2.4, 2.2, 0.85],
                     #           "color": [0, 0, 1, 1], "default_location": "cabinet10_drawer_top"},
                     "bowl": {"type": ObjectType.BOWL, "model": "bowl.stl", "pose": [2.38, 2.2, 1.02],
                              "color": [1, 1, 0, 1], "default_location": "island_countertop"}}


class ContextConfig:
    def __init__(self, context_name, enviornment_name, objects_info):
        self.context_name = context_name
        self.objects_info = objects_info  # A dictionary of object names and their types
        self.environment_name = enviornment_name  # Name of the environment model

    def spawn_objects(self):
        if self.environment_name == "apartment":
            path = "apartment-small.urdf"
        elif self.environment_name == "kitchen":
            path = "kitchen-small.urdf"
        apartment = Object("apartment", ObjectType.ENVIRONMENT, path)

        # Function to spawn objects in the simulation environment
        for obj_name, obj_info in self.objects_info.items():
            obj = Object(obj_name, obj_info['type'], obj_info['model'], pose=Pose(obj_info['pose']),
                         color=obj_info['color'])
            if obj_info['type'] == ObjectType.SPOON:
                apartment.attach(obj, 'cabinet10_drawer_top')

    def search_locations(self, object_name):
        # Function to provide likely locations of objects within this context
        if object_name in self.objects_info:
            return self.objects_info[object_name].get('default_location', 'Unknown')
        return 'Unknown'

    def get_all_objects(self):
        return self.objects_info.keys()


breakfast_context_apartment = ContextConfig("breakfast", "apartment", breakfast_objects)
