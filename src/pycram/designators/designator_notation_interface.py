import inspect
import rospy
import pycram.designators.action_designator as action_designators
import pycram.designators.location_designator as location_designators


# utility functions for mapping of action designators
# returns a list of possible deisgnator names
def get_classes_from_file(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)
    # Filter out only the classes defined in the module
    classes = [member[0] for member in members if
               member[1].__module__ == module.__name__ and "performable" not in member[0].lower()]
    return classes


def autogenerate_class_name_to_class(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)
    # Create a dictionary mapping class names to class objects
    class_name_to_class = {member[0]: member[1] for member in members if member[1].__module__ == module.__name__}
    return class_name_to_class


def get_classes_and_parameters_from_file(module):
    # Get all members of the module
    members = inspect.getmembers(module, inspect.isclass)
    # Filter out only the classes defined in the module that do not have "performable" in their name
    classes = [member for member in members if
               member[1].__module__ == module.__name__ and "performable" not in member[0].lower()]

    class_info = []
    for class_name, class_obj in classes:
        # Get the __init__ method parameters
        init_method = class_obj.__init__
        params = list(inspect.signature(init_method).parameters.keys())
        # Remove 'self' from the parameters
        if 'self' in params:
            params.remove('self')
        class_info.append((class_name, params))

    return class_info


# use in conjunction with get_classes_from_file
def process_action_list_to_dict(action_list):
    processed_dict = {}
    for action in action_list:
        print("Processing action: ", action)
        if action.endswith('Action'):
            new_key = action[:-6]  # Remove 'Action' suffix and convert to lowercase
            processed_dict[new_key] = action
        else:
            processed_dict[action] = action
    return processed_dict


# >>> when called like this gpsr.process_action_list_to_dict(gpsr.get_classes_from_file(actions))
# >>> it returns us a dict shaped like 'transport': 'TransportAction', 'navigate': 'NavigateAction', etc

class ActionDesignator:
    def __init__(self, type=None, **kwargs):
        # Mapping of action types to class names
        action_mapping = process_action_list_to_dict(get_classes_from_file(action_designators))

        # Get the class name from the type
        class_name = action_mapping.get(type)
        if not class_name:
            raise ValueError(f"Unknown action type: {type}")

        # Dynamically create an instance of the class
        class_obj = getattr(action_designators, class_name)
        self.action_instance = class_obj(**kwargs)

    def print_all_parameters(self):
        # Print all parameters of the action instance
        for key, value in self.action_instance.__dict__.items():
            print(f"{key}: {value}")


    def print_parameters(self):
        # Print all parameters of the action instance
        blacklist = ['exceptions', 'executing_thread', 'threads', 'interrupted']
        for key, value in self.action_instance.__dict__.items():
            if key not in blacklist:
                print(f"{key}: {value}")

    def resolve(self):
        # Call the resolve method of the action instance
        for attr_name, attr_value in self.action_instance.__dict__.items():
            # this is location designator specific todo test if only for my locdesig or generally for all of them
            # this is for location designators of type Location(furniture_item='item' ...), so commented out for now.
            #if isinstance(attr_value, location_designators):
            #    tmp = attr_value.ground()
            #    attr_value = tmp.poses
            #    # write target_locations to be the ones from the resolved location designator
            #    self.action_instance.__dict__[attr_name] = attr_value
            #    print("done grounding location designator")
            pass

        return self.action_instance.resolve()

    def perform(self):
        # Call the perform method of the action instance
        self.action_instance.resolve().perform()

    def to_specific_action(self):
        # Return the specific action instance
        return self.action_instance


# currently not used
def create_object_from_string(module, class_name, **kwargs):
    class_name_to_class = autogenerate_class_name_to_class(module)
    class_obj = class_name_to_class.get(class_name)
    if class_obj:
        return class_obj(**kwargs)
    else:
        raise ValueError(f"Unknown class name: {class_name}")
