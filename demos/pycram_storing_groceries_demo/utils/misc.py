import giskardpy
from pycram.designators.action_designator import *
from pycram.designators.motion_designator import *
from pycram.pose import Pose

# poses declared before the challenge
pose_test_area = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_cabinet = Pose([2.25, 2.4, 0], [0, 0, 0, 1])
pose_table = Pose([1.5, 2.2, 0], [0, 0, 1, 0])


def navigate_and_detect(location: str):
    """
    Navigates to given location and perceives.

    :return: tupel of State and dictionary of found objects in the FOV
    """
    TalkingMotion("Navigating").resolve().perform()
    if location == "cabinet":
        NavigateAction([pose_cabinet]).resolve().perform()
    elif location == "table":
        NavigateAction([pose_table]).resolve().perform()

    TalkingMotion("Perceiving").resolve().perform()

    # von Celina und mohammed
    try:
        object_desig = DetectAction(technique='all').resolve().perform()
        giskardpy.sync_worlds()
    except PerceptionObjectNotFound:
        object_desig = {}
    return object_desig


def sort_obj(obj_list: dict):
    """
    sort objects perceived so that knowledge can work with list

    :return: list for knowledge call
    """

    print("not implemented")
    return obj_list
