from pycram.process_module import real_robot
from pycram.designators.action_designator import *
import pycram.external_interfaces.giskard as giskardpy
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

#broadcaster = TFBroadcaster()
#joint_publisher = JointStatePublisher("joint_states", 0.1)

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])
object_mug = BelieveObject(names=["metalmug"])
milk = Object("milk", ObjectType.MILK, "milk.stl", pose=Pose([2.5, 2, 1.02]), color=[1, 0, 0, 1])

# Giskard initialisieren
giskardpy.init_giskard_interface()

with real_robot:
    object_desig = DetectAction(BelieveObject(types=[ObjectType.MILK]), technique='all').resolve().perform()

    PickUpAction(object_designator_description=object_desig["Cerealbox"],
                 arms=["left"],
                 grasps=["front"]).resolve().perform()

    # PickUpAction(object_designator_description=object_desig,
    #          arms=["left"],
    #          grasps=["top"]).resolve().perform()
