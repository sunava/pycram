import rospy
from move_base_msgs.msg import MoveBaseAction
from roslibpy import actionlib

from pycram.designators.action_designator import *
from demos.pycram_receptionist_demo.utils.new_misc import *
from pycram.enums import ObjectType
from pycram.external_interfaces.navigate import PoseNavigator
from pycram.process_module import real_robot
import pycram.external_interfaces.giskard_new as giskardpy
from pycram.ros.robot_state_updater import RobotStateUpdater
from pycram.ros.viz_marker_publisher import VizMarkerPublisher
from pycram.designators.location_designator import *
from pycram.designators.object_designator import *
from pycram.bullet_world import BulletWorld, Object
from std_msgs.msg import String, Bool
from pycram.enums import ImageEnum as ImageEnum
from pycram.utilities.robocup_utils import TextToSpeechPublisher, ImageSwitchPublisher, SoundRequestPublisher, \
    HSRBMoveGripperReal, StartSignalWaiter

world = BulletWorld("DIRECT")
# v = VizMarkerPublisher()
gripper = HSRBMoveGripperReal()
robot = Object("hsrb", "robot", "../../resources/" + robot_description.name + ".urdf")
robot_desig = ObjectDesignatorDescription(names=["hsrb"]).resolve()
robot.set_color([0.5, 0.5, 0.9, 1])

kitchen = Object("kitchen", ObjectType.ENVIRONMENT, "pre_robocup_3.urdf")
print("before giskard")
giskardpy.init_giskard_interface()

couch_pose_semantik = Pose([4, 1.3, 0.8])
RobotStateUpdater("/tf", "/hsrb/robot_state/joint_states")
kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])

# giskardpy.sync_worlds()
move = PoseNavigator()
move.init()
talk = TextToSpeechPublisher()
#talk.init()
# variables for communcation with nlp
pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)
response = ""
callback = False
doorbell = True
timeout = 10  # 15 seconds timeout
laser = StartSignalWaiter()

giskardpy.sync_worlds()
# Declare variables for humans
host = HumanDescription("Vanessa", fav_drink="coffee")
guest1 = HumanDescription("bob", fav_drink="tea")
# for testing, if the first part of the demo is skipped
guest1.set_attributes(['male', 'without a hat', 'wearing a t-shirt', ' a dark top'])
guest1.set_id(9)

guest2 = HumanDescription("Sarah", fav_drink="Juice")
# for testing, if the first part of the demo is skipped
guest2.set_attributes(['female', 'with a hat', 'wearing a t-shirt', ' a bright top'])

image_switch_publisher = ImageSwitchPublisher()




def data_cb(data):
    global response
    global callback
    global doorbell

    image_switch_publisher.pub_now(ImageEnum.HI.value)
    response = data.data.split(",")
    response.append("None")
    callback = True


def doorbell_cb(data):
    global doorbell
    doorbell = True


def pakerino(torso_z=0.05, config = None):

    if not config:
        config = {'arm_lift_joint': torso_z, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': 0}

    giskardpy.avoid_all_collisions()
    giskardpy.achieve_joint_goal(config)
    print("Parking done")

def door_opening():
    config = {'arm_lift_joint': 0.4, 'arm_flex_joint': 0, 'arm_roll_joint': -1.2, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': -1.57}

    pakerino(config=config)
    # pose1 = Pose([1.35, 4.4, 0], [0, 0, 1, 0])
    pose1 = Pose([1, 4, 0], [0, 0, 1, 0])
    move.pub_now(pose1)
    
    # grasp handle
    talk.pub_now("grasp handle now", True)
    gripper.pub_now("open")
    giskardpy.grasp_doorhandle("kitchen_2/living_room:arena:door_handle_inside")
    gripper.pub_now("close")

    # open door
    talk.pub_now("open dor now", True)
    giskardpy.open_doorhandle("kitchen_2/living_room:arena:door_handle_inside")
    gripper.pub_now("open")

    door_open_bool = laser.something_in_the_way()
    if door_open_bool:
        talk.pub_now("I opened the door")
    else:
        talk.pub_now("I made a mistake. Pleae open the door yourself")
    # Arm to save position
    talk.pub_now("park arms", True)
    arm_to_left_config = {'arm_lift_joint': 0.4, 'arm_flex_joint': 0, 'arm_roll_joint': 1.5, 'wrist_flex_joint': -1.5,
              'wrist_roll_joint': -1.57}
    pakerino(config=arm_to_left_config)

    # move away from door
    talk.pub_now("move away from door", True)
    pose2 = Pose([1.9, 4.5, 0], [0, 0, 1, 0])
    move.pub_now(pose2)


def get_attributes(guest: HumanDescription):
    """
    storing attributes and face of person in front of robot
    :param guest: variable to store information in
    """
    # remember face
    keys = DetectAction(technique='human', state='face').resolve().perform()[1]
    new_id = keys["keys"][0]
    guest1.set_id(new_id)

    # get clothes and gender
    attr_list = DetectAction(technique='attributes', state='start').resolve().perform()
    guest1.set_attributes(attr_list)
    rospy.loginfo(attr_list)


def welcome_guest(num, guest: HumanDescription):
    """
    talking sequence to get name and favorite drink of guest
    and attributes if it is the first guest
    :param num: number of guest
    :param guest: variable to store information in
    """
    global callback

    talk.pub_now("Welcome, please step in front of me")

    # look for human
    DetectAction(technique='human').resolve().perform()

    # look at guest and introduce
    giskardpy.move_head_to_human()

    talk.pub_now("Hello, i am Toya and my favorite drink is oil", True)

    rospy.sleep(3)
    talk.pub_now("please answer me after the sound", True)

    rospy.sleep(2)
    talk.pub_now("What is your name and favorite drink?", True)

    rospy.sleep(2)

    # signal to start listening
    pub_nlp.publish("start listening")
    rospy.sleep(2.3)
    image_switch_publisher.pub_now(ImageEnum.TALK.value)


    # wait for nlp answer
    start_time = time.time()
    while not callback:
        rospy.sleep(1)

        if time.time() - start_time == timeout:
            print("guest needs to repeat")
            image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
    callback = False

    if response[0] == "<GUEST>":
        # success a name and intent was understood
        if response[1] != "<None>":
            guest.set_drink(response[2])
            rospy.sleep(1)
            guest.set_name(response[1])

        else:
            # save heard drink
            if response[2]:
                guest.set_drink(response[2])

            # ask for name again once
            guest.set_name(name_repeat())

        if response[2] != "<None>":
            # confirm favorite drink
            guest.set_drink(response[2])
        else:
            print("drink repeat todo")

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            talk.pub_now("please repeat your name and drink loud and clear", True)
            # TalkingMotion("please repeat your name and drink loud and clear").resolve().perform()
            pub_nlp.publish("start")
            rospy.sleep(2)
            image_switch_publisher.pub_now(ImageEnum.TALK.value)


            start_time = time.time()
            while not callback:
                rospy.sleep(1)
                if time.time() - start_time == timeout:
                    print("guest needs to repeat")
                    image_switch_publisher.pub_now(ImageEnum.JREPEAT.value)
            callback = False

            if response[0] == "<GUEST>":
                guest.set_name(response[1])
                guest.set_drink(response[2])
                break
            else:
                i += 1

    # get attributes and face if first guest
    if num == 1:
        try:
            talk.pub_now("please wait and look at me")
            get_attributes(guest)

        except PerceptionObjectNotFound:
            # failure handling, if human has stepped away
            talk.pub_now("please step in front of me")
            rospy.sleep(3.5)

            try:
                get_attributes(guest)

            except PerceptionObjectNotFound:
                print("continue without attributes")

    talk.pub_now("i will show you the living room now")

    print(guest.id)
    return guest


def detect_point_to_seat():
    """
    function to look for a place to sit and poit to it
    returns bool if free place found or not
    """

    # detect free seat
    seat = DetectAction(technique='location', state="sofa").resolve().perform()
    free_seat = False
    # loop through all seating options detected by perception
    for place in seat[1]:
        if place[0] == 'False':

            not_point_poe = Pose([float(place[1]), float(place[2]), 0.85])
            print("place: "+ str(place))

            lt = LocalTransformer()
            pose_in_robot_frame = lt.transform_pose(not_point_poe, robot.get_link_tf_frame("base_link"))
            print("in robot: " + str(pose_in_robot_frame))
            if pose_in_robot_frame.pose.position.y > 0.05:
                talk.pub_now("left from me")
                pose_in_robot_frame.pose.position.y += 0.8

            else:
                talk.pub_now("right from me")
                pose_in_robot_frame.pose.position.y -= 0.8
            pose_in_map = lt.transform_pose(pose_in_robot_frame, "map")


            free_seat = True
            pose_guest = PointStamped()
            pose_guest.header.frame_id = "map"
            pose_guest.point.x = pose_in_map.pose.position.x
            pose_guest.point.y = pose_in_map.pose.position.y
            pose_guest.point.z = 0.85
            print(pose_guest)

            giskardpy.move_arm_to_point(pose_guest)

            print("found seat")
            return pose_guest
    return free_seat


def demo(step):
    with real_robot:
        global callback
        global doorbell
        global guest1
        global guest2
        pakerino()
        # signal start
        rospy.logerr("here")
        talk.pub_now("waiting for guests")
        image_switch_publisher.pub_now(ImageEnum.HI.value)

        # receive data from nlp via topic
        rospy.Subscriber("nlp_out", String, data_cb)
        rospy.Subscriber("nlp_out2", String, doorbell_cb)

        if step <= 0:
            # door opening sequence

            # wait 15 seconds for sound
            start_time = time.time()
            while not doorbell:
                # continue challenge to not waste time
                if time.time() - start_time > timeout:
                    print("Timeout reached, no bell")
                    break

                time.sleep(0.5)

            # set it back to false for second guest
            doorbell = False

            #door_opening()

        if step <= 1:

            # reception/talking sequence
            pakerino()
            guest1 = welcome_guest(1, guest1)

        if step <= 2:
            # leading to living room and pointing to free seat
            talk.pub_now("please step out of the way and follow me")

            # stop looking at human
            giskardpy.cancel_all_called_goals()
            DetectAction(technique='human', state='stop').resolve().perform()

            # lead human to living room
            move.pub_now(door_to_couch)
            gripper.pub_now("close")

            # place new guest in living room
            talk.pub_now("welcome to the living room")
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)

            # detect host
            host_pose = DetectAction(technique='human').resolve().perform()
            host.set_pose(host_pose[1])
            DetectAction(technique='human', state='stop').resolve().perform()

            # find free seat
            guest_pose = detect_point_to_seat()
            if not guest_pose:
                # move head a little to perceive chairs
                config = {'head_pan_joint': -0.3}
                pakerino(config)
                guest_pose = detect_point_to_seat()
                guest1.set_pose(guest_pose)
            else:
                guest1.set_pose(guest_pose)

        if step <= 3:
            # introduce guest1 and host
            giskardpy.move_head_to_human()
            rospy.sleep(1)
            if guest1.pose:
                pub_pose.publish(guest1.pose)

            talk.pub_now("please take a seat next to your host")
            # image_switch_publisher.pub_now(ImageEnum.SOFA.value)

            # try:
            #     keys = DetectAction(technique='human', state='face').resolve().perform()[1]
            #     host.set_id(keys["keys"][0])
            #
            # except PerceptionObjectNotFound:
            #     print("no id found")

            rospy.sleep(1)

            # introduce humans and look at them
            introduce(host, guest1)
            rospy.sleep(2)

        if step <= 4:
            # signal start
            talk.pub_now("waiting for new guests")
            image_switch_publisher.pub_now(ImageEnum.HI.value)

            # wait 10 seconds for sound
            start_time = time.time()
            while not doorbell:
                # continue challenge to not waste time
                if time.time() - start_time > timeout:
                    print("Timeout reached, no bell")
                    break

                time.sleep(0.5)

            # TODO: Giskard has to fix world state
            # door_opening()

            pose2 = Pose([1.85, 4.5, 0], [0, 0, 1, 0])
            move.pub_now(pose2)

        if step <= 5:
            guest2 = welcome_guest(2, guest2)

        if step <= 6:
            # leading to living room and pointing to free seat
            talk.pub_now("please step out of the way and follow me")

            # stop looking at human
            giskardpy.cancel_all_called_goals()
            DetectAction(technique='human', state='stop').resolve().perform()

            # lead human to living room
            move.pub_now(door_to_couch)

            talk.pub_now("welcome to the living room")
            giskardpy.move_head_to_pose(couch_pose_semantik)
            rospy.sleep(1)
            gripper.pub_now("close")
            # place new guest in living room

        if step <= 7:
            # update poses from guest1 and host
            counter = 0
            while True:
                try:
                    human_dict = DetectAction(technique='human', state='face').resolve().perform()[1]
                    counter += 1
                    id_humans = human_dict["keys"]
                    print("id humans: " + str(id_humans))

                    found_guest = False
                    for key in id_humans:
                        print("key: " + str(key))
                        if key == guest1.id:
                            guest1_pose = human_dict[guest1.id]
                            talk.pub_now("found guest")
                            found_guest = True
                            # guest1.set_pose(guest1_pose)
                            break
                        else:
                            print("nothing to see here")
                            # host_pose = id_humans[key]
                            # host.set_pose(host_pose)

                    if found_guest or counter > 3:
                        break
                except PerceptionObjectNotFound:
                     print("i am a faliure and i hate my life")

            if not found_guest:
                try:
                    guest1.set_pose(human_dict[id_humans[0]])
                    host.set_pose(human_dict[id_humans[1]])
                except Exception as e:
                    print(e)




        if step <= 8:
            talk.pub_now("look at couch seat")
            # TODO: Look at couch does not work
            giskardpy.move_head_to_pose(couch_pose_semantik)

            # find a place for guest2 to sit and point
            guest_pose = detect_point_to_seat()
            talk.pub_now("take a seat")
            #image_switch_publisher.pub_now(ImageEnum.SOFA.value)

            if not guest_pose:
                # move head a little
                # TODO: failure handling for couch seat
                config = {'head_pan_joint': -0.3}
                pakerino(config)
                guest_pose = detect_point_to_seat()
                if guest_pose:
                    guest2.set_pose(guest_pose)
            else:
                guest2.set_pose(guest_pose)

            if step <= 9:
                # introduce everyone to guest 2
                giskardpy.move_head_to_human()
                introduce(host, guest2)
                rospy.sleep(3)
                introduce(guest1, guest2)
                rospy.sleep(3)


demo(5)