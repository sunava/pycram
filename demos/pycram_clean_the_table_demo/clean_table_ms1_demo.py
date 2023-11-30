import rospy

from pycram.process_module import simulated_robot, real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.external_interfaces import robokudo
from pycram.designators.object_designator import *
from pycram.enums import ObjectType
#from pycram.ros.tf_broadcaster import TFBroadcaster

from pycram.ros.robot_state_updater import RobotStateUpdater
#from pycram.ros.joint_state_publisher import JointStatePublisher


world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

# TODO: Was bewirken diese Zeilen Code?

#broadcaster = TFBroadcaster()
#joint_publisher = JointStatePublisher("joint_states", 0.1)
world.set_gravity([0, 0, -9.8])

# Calculate Quaternion for a 180-degree turn of the robot
robo_orientation = axis_angle_to_quaternion([0, 0, 1], 180)
x = robo_orientation[0]
y = robo_orientation[1]
z = robo_orientation[2]
w = robo_orientation[3]

# Initialize objects in BulletWorld
# TODO: Warum ist die Orientierung vom falschen Datentyp? Kann ich das ignorieren?
robot = Object("hsrb", ObjectType.ROBOT, "../../resources/" + "hsrb" + ".urdf")
robot.set_color([0.5, 0.5, 0.9, 1])
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
milk = Object("milk", "milk", "milk.stl", pose=Pose([-0.7841, 2.1089, 0.9]), color=[1, 0, 0, 1])
cereal = Object("cereal", "cereal", "breakfast_cereal.stl", pose=Pose([-0.7841, 2.3, 0.9]), color=[0, 1, 0, 1])
cereal2 = Object("cereal2", "cereal", "breakfast_cereal.stl", pose=Pose([-0.7841, 2.6, 0.9]), color=[0, 1, 0, 1])
milk2 = Object("milk2", "milk", "milk.stl", pose=Pose([-0.7841, 1.8, 0.9]), color=[0, 1, 0, 1])

milk_desig = BelieveObject(names=["milk"])
cereal_desig = BelieveObject(names=["cereal"])
cereal2_desig = BelieveObject(names=["cereal2"])
milk2_desig = BelieveObject(names=["milk2"])

# Liste mit allen BelieveObjects
# TODO: Brauchen wir BelieveObjects auch für den real_robot?
object_list = [milk2_desig, milk_desig, cereal_desig, cereal2_desig]

# Liste aller Objektpositionen
robot_pose_list = [Pose([-0.7841, 1.8, 0.9]), Pose([-0.7841, 2.1089, 0.9]), Pose([-0.7841, 2.3, 0.9]), Pose([-0.7841, 2.6, 0.9])]
# Giskard initialisieren und syncen
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()

RobotStateUpdater("/tf", "/giskard_joint_states")
# Eventuell Liste der Objekttypen
# object_type_list = [ObjectType.MILK, ObjectType.MILK, ObjectType.BREAKFAST_CEREAL, ObjectType.BREAKFAST_CEREAL]


# with simulated_robot:
#     ParkArmsAction([Arms.LEFT]).resolve().perform()
#     MoveTorsoAction([0.33]).resolve().perform()
#
#     for index in range(len(object_list)):
#         #LookAtAction(targets=[robot_pose_list[index]]).resolve().perform()
#         #object_desig = DetectAction(BelieveObject(types=[object_type_list[index])).resolve().perform()
#         # dann würde hier die Liste der object_list wegfallen
#         PickUpAction(object_designator_description=object_list[index], arms=["left"], grasps=["front"]).resolve().perform()
#         PlaceAction(object_list[index], [robot_pose_list[index]], ["left"]).resolve().perform()
#         ParkArmsAction([Arms.LEFT]).resolve().perform()
#         NavigateAction(target_locations=[Pose([robot_pose_list[index].pose.x, robot_pose_list[index].pose.y + 0.7, robot_pose_list[index].pose.z])]).resolve().perform()

with real_robot:
    # Arme korrekt positionieren
    ParkArmsAction([Arms.LEFT]).resolve().perform()

    NavigateAction(target_locations=[Pose([-0.2, 1.8, 0.9], robo_orientation)]).resolve().perform()


    # Torso hochfahren, für einen besseren Blick
    MoveTorsoAction([0.4]).resolve().perform()


    print("1")
    # TODO: perceive Pose für Perception einnehmen mittels LookAtAction? Auf welche Position schauen?
    LookAtAction(targets=[Pose([-0.7841, 1.8, 0.9]), Pose([-0.7841, 2.1089, 0.9]), Pose([-0.7841, 2.3, 0.9]), Pose([-0.7841, 2.6, 0.9])]).resolve().perform()

    print("2")

    # Beschreibungen der 5 wahrzunehmenden Objekttypen
    # TODO: kann ich hier eine Liste übergeben? Warum kann man nicht einfach all sagen?
   # object_desig_desc = ObjectDesignatorDescription(types=["ObjectType.MILK", "ObjectType.MILK", "ObjectType.BREAKFAST_CEREAL", "ObjectType.BREAKFAST_CEREAL"])

    # gibt ein dictionary von Poses/PoseStamped??, der erkannten Objekte zurück
    # Query Aufruf von Perception
   # object_pose_list = robokudo.query(object_desig_desc)

    # Durchlaufen aller erkannten Objekte
    for index in range(4):#len(object_list)):
        print("3")
        # Position zum Bewegen einnehmen
        ParkArmsAction([Arms.LEFT]).resolve().perform()
        print("4")
        # TODO: Sollte hier statt die object_list die object_pose_list verwendet werden?
        # Position für den Roboter definieren, von der er das nächste Objekt aufnehmen kann
        #pickup_pose_object = CostmapLocation(target=object_list[index].resolve(), reachable_for=robot_desig).resolve()
        print("5")
        # Den Roboter an dieser Position navigieren
        #NavigateAction(target_locations=[pickup_pose_object.pose]).resolve().perform()

        print("angekommen")
        # Aufnehmen des Objektes, erstmal alle von der front, später ergänzen
        PickUpAction(object_designator_description=object_list[index],
                     arms=["left"],
                     grasps=["front"]).resolve().perform()
        print("hier")
        # drop object, durch öffnen des Grippers. oder hand_palm_link?
        # TODO: vorher textausgabe, mittels subscriber auf /chatter oder /speech?
        MoveGripperMotion("open", "arm" , True)
        print("end")
        # TODO: wie mit object_pose_list arbeiten
        rospy.spin()




#manipulation perceive pose
#perceived_items[objectdesig] = perception call
#     spawn perceived_items
#     syn.world giskard
#     add.mesh to giskard
#     loop for items
#        wenn items[i] == item1 dann
#         item1 = items[i]
#        ...
#      item1 pick up
#      drop item / place item1 in simulation
#      ...
#
