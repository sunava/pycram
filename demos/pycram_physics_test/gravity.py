from pycram.process_module import simulated_robot, with_simulated_robot
from pycram.designators.action_designator import *
from pycram.enums import Arms, ObjectType
from pycram.designators.object_designator import *
from pycram.designators.object_designator import BelieveObject
import pycram.helper as helper

world = BulletWorld()
#print(world.shadow_world)

# robot_desig = ObjectDesignatorDescription(names=["pr2"]).resolve()
kitchen = Object("kitchen", "environment", "kitchen.urdf")
# robot.set_joint_state(robot_description.torso_joint, 0.24)
# kitchen_desig = ObjectDesignatorDescription(names=["kitchen"])
spawning_poses = {
    # 'bigknife': Pose([-0.95, 1.2, 1.3], [1, -1, 1, -1]),
    'bigknife': Pose([0.9, 0.6, 0.8], [0, 0, 0, -1]),
    # 'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, 1])
    'bread': Pose([-0.85, 0.9, 0.90], [0, 0, -1, -1]),
    'board': Pose([-0.85, 0.9, 0.85], [0, 0, -1, -1]),
    'cocumber': Pose([-0.85, 0.9, 1], [0, 0, -1, -1])
}
cocumber = Object("cocumber", "cocumber", "cocumber.stl", spawning_poses["cocumber"])
board = Object("board", "board", "board.stl", spawning_poses["board"])
cocumber.set_color([0, 1, 0.04, 1])
#falling down
#world.simulate(1)

robot = Object("pr2", "robot", "../../resources/" + robot_description.name + ".urdf")


world.set_realtime(True)
world.get_physic_world().set_gravity([0, 0, -9.81])