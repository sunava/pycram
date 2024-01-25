from pycram.process_module import real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

#broadcaster = TFBroadcaster()
#joint_publisher = JointStatePublisher("joint_states", 0.1)

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
robot_pose= robot.get_pose()
object_mug = BelieveObject(names=["metalmug"])
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])

# Giskard initialisieren
giskardpy.init_giskard_interface()
RobotStateUpdater("/tf", "/giskard_joint_states")

def try_pick_up(obj, grasps):
    try:
        PickUpAction(obj, ["left"], [grasps]).resolve().perform()
    except (EnvironmentUnreachable, GripperClosedCompletely):
        print("try pick up again")
        TalkingMotion("Try pick up again")
        NavigateAction([Pose([robot_pose.position.x - 0.3, robot_pose.position.y, robot_pose.position.z],
                             robot_pose.orientation)]).resolve().perform()
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        if EnvironmentUnreachable:
             object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()
             # TODO nur wenn key (name des vorherigen objektes) in object_desig enthalten ist
             new_object = object_desig[obj.name]
        else:
             new_object = obj
        try:
            PickUpAction(new_object, ["left"], [grasps]).resolve().perform()

        except:
            TalkingMotion(f"Can you pleas give me the {obj.name} object on the table? Thanks")
            TalkingMotion(f"Please push down my hand, when I can grab the {obj.name}.")


with real_robot:
    ParkArmsAction([Arms.LEFT]).resolve().perform()
    #object_desig = DetectAction(BelieveObject(types=[ObjectType.METALMUG])).resolve().perform()
    object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()
   # PickUpAction(object_designator_description=object_desig,
     #            arms=["left"],
      #           grasps=["top"]).resolve().perform()
    giskardpy.initial_adding_objects()
    giskardpy.spawn_kitchen()

    #try_pick_up(object_desig["Cerealbox"], "front")
    #todo: macht avoid collision oder allow ueberhaupt was? muessten wir testen
    #giskardpy.place_objects(object_desig["Cerealbox"],[Pose([4.82,1.8,0.73])])
    #todo: in dem designator noch pose bearbeiten jenachdem welches object_desig reinkommt bei cereal großen z bei spoon minimal z
    # und atm ist nur front place drin neues argument how to place

    PlaceAction(object_desig["Cerealbox"], [Pose([4.92,1.8,0.74])], ["left"], ["front"]).resolve().perform()
