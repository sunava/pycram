from pycram.process_module import simulated_robot, with_simulated_robot, real_robot, with_real_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper
import pycram.external_interfaces.giskard as giskardpy
from pycram.failure_handling import Retry
from pycram.ros.viz_marker_publisher import VizMarkerPublisher

from pycram.language import macros, par
import sys
print(sys.meta_path)
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
world = BulletWorld("DIRECT")
v = VizMarkerPublisher()

world.set_gravity([0, 0, -9.8])
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
robot.set_joint_state(robot_description.torso_joint, 0.24)
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
spawning_poses = {
    # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
    'bigknife': Pose([0.9, 0.6, 0.5], [0, 0, 0, -1]),
    # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 0.87], [0, 0, -1, -1])
}
bigknife = Object("bigknife", "bigknife", "big-knife.stl", spawning_poses["bigknife"])
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
board.set_color([0.4, 0.2, 0.06, 1])
bigknife_BO = BelieveObject(names=["bigknife"])
bread_BO = BelieveObject(names=["bread"])
cocumber_BO = BelieveObject(names=["cocumber"])
giskardpy.init_giskard_interface()
giskardpy.sync_worlds()


class HumanDescription(name, tshirtcolor, favedrink, gender, age):
    ..
    self.name = name

with simulated_robot:
    host = humandescription(vanessa,.....)
    human1 = human....
    if topic true ...
     #hardgecoded erstmal die drawer um die action drin zu haben
    OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
    #texttospeechcall:"hallo freund, mein favorite drink is oil what is yours"
            par (

                if (human1)
                     LookAtAction(targets=[human1.pose]).resolve().perform()
                     wait for 10 (sec..)
        
               #          query fav drink name
               #      "PLEASDE SAY SSOMETHING"  
               #          back to loop wait for
               # #future optional  1.0 2 query2 = query to selfmade topic -->


            )
            human1 = human1 + human1.favdrink=blah
            # par (
                 #texttospeechcall: "please follow me into the living room"
                 #stop perception publish )
               #(
                   par
                DetectAction(cocumber_BO).resolve().perform()
                pose = query knowledge living room / harcdcoded pose
                NavigateAction(target_locations=[pipose]).resolve().perform()
                hostpose = perceptioncall -> find host der hat kein gruenes tshirt an .. (nicht human1) -> pose
                     percepthumanquery(human1.tshirtcolor,....)
                sittingpose = perceptioncall free spot for sitting.pose
                 )
  par( LookAtAction(targets=[pick_pose]).resolve().perform()
texttospeechcall: "es gibt ein freien sitzplatz vor mir links neben hostg 2")
            par (manipulation move head to hostpose
                    texttospeechcall: "hallo host ich stelle dir x vor er hat b....."
                    manipulation move head to sittingpose
                    texttospeechcall: "host is..."
                    )




##das ist die ganze logik von der challange mit echten roboter und menschis
# class HumanDescription(name, tshirtcolor, favedrink, gender, age):
#     ..
#     self.name = name
#
# with simulated_robot:
#     host = humandescription(vanessa,.....)
#     wait for / if  (Query -> knowledge ->  doorbell bool): fake: bool true
#             handle desig = link von door knauf
#             OpenAction(object_designator_description=handle_desig, arms=[drawer_open_loc.arms[0]]).resolve().perform()
#     texttospeechcall:"hallo freund, mein favorite drink is oil what is yours"
#             par (
#
#                 if (perception call = perception -> human erkannt
#                   DetectAction(cocumber_BO).resolve().perform())
#                       manipulation -> look at human
#                 2 query2 = query to knowledge -> human1 description
#
#             )
#             human 1 = humandescription(query2.name, perception.age)
#              par (
#                  texttospeechcall: "please follow me into the living room"
#                  stop perception publish )
#                (par
#                 pose = query knowledge living room / harcdcoded pose
#                 NavigateAction(target_locations=[pipose]).resolve().perform()
#                 hostpose = perceptioncall -> find host der hat kein gruenes tshirt an .. (nicht human1) -> pose
#                      percepthumanquery(human1.tshirtcolor,....)
#                 sittingpose = perceptioncall free spot for sitting.pose
#                  )
#   par( manipulation move head to sittingpose
# texttospeechcall: "es gibt ein freien sitzplatz vor mir links neben hostg 2")
#             par (manipulation move head to hostpose
#                     texttospeechcall: "hallo host ich stelle dir x vor er hat b....."
#                     manipulation move head to sittingpose
#                     texttospeechcall: "host is..."
#                     )



