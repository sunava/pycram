from ..robot_description import *


class ARMAR6Description(RobotDescription):

    def __init__(self):
        super().__init__("Armar6", "base_footprint", "platform", "torso", "torso_joint")
        # Camera
        depth_camera = CameraDescription("DepthCamera",
                                         horizontal_angle=0.99483, vertical_angle=0.75049)
        flea_cameras_center = CameraDescription("FleaCamerasCenter",
                                                horizontal_angle=0.99483, vertical_angle=0.75049)
        roboception = CameraDescription("Roboception",
                                        horizontal_angle=0.99483, vertical_angle=0.75049)
        self.add_cameras({"depth_camera": depth_camera, "flea_cameras_center": flea_cameras_center,
                          "roboception": roboception})
        # The axis which points away from the camera and along which the picture of the camera is created
        self.front_facing_axis = [0, 0, 1]
        # Neck
        neck_links = ["middle_neck", "upper_neck"]
        neck_joints = ["neck_1_yaw", "neck_2_pitch"]
        neck_forward = {"forward": [0.0, 0.0], "down": [0.0, 0, 0]}
        neck_chain = ChainDescription("neck", neck_joints, neck_links, static_joint_states=neck_forward)
        self.add_chain("neck", neck_chain)
        # Arm
        arm_l_joints = ["arm_t12_joint_r0", "arm_t23_joint_r0", "arm_t34_joint_r0", "arm_t45_joint_r0",
                        "arm_t56_joint_r0", "arm_t67_joint_r0", "arm_t78_joint_r0", "arm_t8_joint_r0"]
        arm_l_links = ["arm_cla_r0", "arm_t12_r0", "arm_t23_r0", "arm_t34_r0", "arm_t45_r0", "arm_t56_r0", "arm_t67_r0",
                       "arm_t78_r0", "arm_t8_r0", "Hand L Palm", "Index L 1", "Index L 2", "Index L 3", "Middle L 1",
                       "Middle L 2", "Middle L 3", "Ring L 1", "Ring L 2", "Ring L 3", "Pinky L 1", "Pinky L 2",
                       "Pinky L 3", "Thumb L 1", "Thumb L 2"]
        arm_r_joints = ["arm_t12_joint_r1", "arm_t23_joint_r1", "arm_t34_joint_r1", "arm_t45_joint_r1",
                        "arm_t56_joint_r1", "arm_t67_joint_r1", "arm_t78_joint_r1", "arm_t8_joint_r1"]
        arm_r_links = ["arm_cla_r1", "arm_t12_r1", "arm_t23_r1", "arm_t34_r1", "arm_t45_r1", "arm_t56_r1", "arm_t67_r1",
                       "arm_t78_r1", "arm_t8_r1", "Hand R Palm", "Index R 1", "Index R 2", "Index R 3", "Middle R 1",
                       "Middle R 2", "Middle R 3", "Ring R 1", "Ring R 2", "Ring R 3", "Pinky R 1", "Pinky R 2",
                       "Pinky R 3", "Thumb R 1", "Thumb R 2"]
        gripper_l_joints = ["Thumb L 1 Joint", "Thumb L 2 Joint", "Index L 1 Joint", "Index L 2 Joint",
                            "Index L 3 Joint", "Middle L 1 Joint", "Middle L 2 Joint", "Middle L 3 Joint",
                            "Ring L 1 Joint", "Ring L 2 Joint", "Ring L 3 Joint", "Pinky L 1 Joint", "Pinky L 2 Joint",
                            "Pinky L 3 Joint"]
        gripper_l_links = ["Hand L Palm", "Index L 1", "Index L 2", "Index L 3", "Middle L 1", "Middle L 2",
                           "Middle L 3", "Ring L 1", "Ring L 2", "Ring L 3", "Pinky L 1", "Pinky L 2", "Pinky L 3",
                           "Thumb L 1", "Thumb L 2"]
        gripper_r_joints = ["Thumb R 1 Joint", "Thumb R 2 Joint", "Index R 1 Joint", "Index R 2 Joint",
                            "Index R 3 Joint", "Middle R 1 Joint", "Middle R 2 Joint", "Middle R 3 Joint",
                            "Ring R 1 Joint", "Ring R 2 Joint", "Ring R 3 Joint", "Pinky R 1 Joint", "Pinky R 2 Joint",
                            "Pinky R 3 Joint"]
        gripper_r_links = ["Hand R Palm", "Index R 1", "Index R 2", "Index R 3", "Middle R 1",
                           "Middle R 2", "Middle R 3", "Ring R 1", "Ring R 2", "Ring R 3", "Pinky R 1", "Pinky R 2",
                           "Pinky R 3", "Thumb R 1", "Thumb R 2"]
        arm_l_park = {"park": [0, 0, 1.5, 0.5, 2.0, 1.5, 0, 0]}
        arm_r_park = {"park": [0, 0, 1.5, 2.64, 2.0, 1.6415, 0, 0]}
        gripper_l = GripperDescription("gripper_l", gripper_links=gripper_l_links, gripper_joints=gripper_l_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_l_chain = ChainDescription("left", arm_l_joints, arm_l_links, static_joint_states=arm_l_park)
        arm_l_inter = InteractionDescription(arm_l_chain, "arm_t8_r0")
        arm_l_manip = ManipulatorDescription(arm_l_inter, tool_frame="left_tool_frame",
                                             gripper_description=gripper_l)

        gripper_r = GripperDescription("gripper_r", gripper_links=gripper_r_links, gripper_joints=gripper_r_joints,
                                       gripper_meter_to_jnt_multiplier=16.0, gripper_minimal_position=1.57,
                                       gripper_convergence_delta=0.005)
        arm_r_chain = ChainDescription("right", arm_r_joints, arm_r_links, static_joint_states=arm_r_park)
        arm_r_inter = InteractionDescription(arm_r_chain, "arm_t8_r1")
        arm_r_manip = ManipulatorDescription(arm_r_inter, tool_frame="right_tool_frame",
                                             gripper_description=gripper_r)

        self.add_chains({"left": arm_l_manip, "right": arm_r_manip})
        self.add_static_gripper_chains("left", {"open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                                "close": [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57,
                                                          1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, ]})

        self.add_static_gripper_chains("right", {"open": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                                                "close": [1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57,
                                                          1.57, 1.57, 1.57, 1.57, 1.57, 1.57, 1.57, ]})

        self.grasps = GraspingDescription(
            {"front": [0.707, 0.707, 0.707, 0.707],
             "left": [1, 0, 0, 1],
             "right": [0, 1, 1, 0],
             "top": [-1, 0, 0, 0]})

    def get_camera_frame(self, name="roboception"):
        # TODO: Hacky since only one optical camera frame from pr2 is used
        return super().get_camera_frame(name)
