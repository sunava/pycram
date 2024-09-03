import atexit
import time
import threading
from typing import List

import rospy
import pybullet as pb

from pycram.worlds.bullet_world import BulletWorld
from geometry_msgs.msg import WrenchStamped
from std_msgs.msg import Header

from .custom_filter import Butterworth

from pycram.datastructures.enums import FilterConfig


class ForceTorqueSensorSimulated:
    """
    Simulated force-torque sensor for a joint with a given name.
    Reads simulated forces and torques at that joint from bullet_world and publishes geometry_msgs/Wrench messages
    to the given topic.
    """

    def __init__(self, joint_name, fts_topic="/pycram/fts", interval=0.1):
        """
        The given joint_name has to be part of :py:attr:`~pycram.bullet_world.BulletWorld.robot` otherwise a
        RuntimeError will be raised.

        :param joint_name: Name of the joint for which force-torque should be simulated
        :param fts_topic: Name of the ROS topic to which should be published
        :param interval: Interval at which the messages should be published, in seconds
        """
        self.world = BulletWorld.current_bullet_world
        self.fts_joint_idx = None
        self.joint_name = joint_name
        if joint_name in self.world.robot.joints.keys():
            self.fts_joint_idx = self.world.robot.joints[joint_name]
        else:
            raise RuntimeError(
                f"Could not register ForceTorqueSensor: Joint {joint_name} does not exist in robot object")
        pb.enableJointForceTorqueSensor(self.world.robot.id, self.fts_joint_idx, enableSensor=1)

        self.fts_pub = rospy.Publisher(fts_topic, WrenchStamped, queue_size=10)
        self.interval = interval
        self.kill_event = threading.Event()

        self.thread = threading.Thread(target=self._publish)
        self.thread.start()

        atexit.register(self._stop_publishing)

    def _publish(self) -> None:
        """
        Continuously publishes the force-torque values for the simulated joint. Values are published as long as the
        kill_event is not set.
        """
        seq = 0
        while not self.kill_event.is_set():
            current_joint_state = pb.getJointState(self.world.robot.id, self.fts_joint_idx)
            joint_ft = current_joint_state[2]
            h = Header()
            h.seq = seq
            h.stamp = rospy.Time.now()
            h.frame_id = self.joint_name

            wrench_msg = WrenchStamped()
            wrench_msg.header = h
            wrench_msg.wrench.force.x = joint_ft[0]
            wrench_msg.wrench.force.y = joint_ft[1]
            wrench_msg.wrench.force.z = joint_ft[2]

            wrench_msg.wrench.torque.x = joint_ft[3]
            wrench_msg.wrench.torque.y = joint_ft[4]
            wrench_msg.wrench.torque.z = joint_ft[5]

            self.fts_pub.publish(wrench_msg)
            seq += 1
            time.sleep(self.interval)

    def _stop_publishing(self) -> None:
        """
        Sets the kill_event and therefore terminates the Thread publishing the force-torque values as well as join the
        threads.
        """
        self.kill_event.set()
        self.thread.join()


class ForceTorqueSensor:
    """
    Monitor a force-torque sensor of a supported robot and save relevant data.

    Apply a specified filter and save this data as well.
    Default filter is the low pass filter 'Butterworth'

    Can also calculate the derivative of (un-)filtered data
    """
    filtered = 'filtered'
    unfiltered = 'unfiltered'

    def __init__(self, robot_name, filter_config=FilterConfig.butterworth, filter_order=4, custom_topic=None,
                 debug=False):

        """
        Create a subscriber for the force-torque-sensor topic of a specified robot.

        :param robot_name: Name of the robot
        :param filter_config: Desired filter (default: Butterworth)
        :param filter_order: Order of the filter. Declares the number of elements that delay the sampling
        :param custom_topic: Declare a custom topic if the default topics do not fit
        """

        self.robot_name = robot_name
        self.filter_config = filter_config
        self.filter = self.__get_filter(order=filter_order)
        self.debug = debug

        self.wrench_topic_name = custom_topic
        self.force_torque_subscriber = None
        self.init_data = True

        self.whole_data = None
        self.prev_values = None

        self.order = filter_order

        self.__setup()

    def __setup(self):
        self.__get_robot_parameters()
        self.subscribe()

    def __get_robot_parameters(self):
        if self.wrench_topic_name is not None:
            return

        if self.robot_name == 'hsrb':
            self.wrench_topic_name = '/hsrb/wrist_wrench/compensated'
            # arm_topic_name = '/hsrb/arm_trajectory_controller/command'
            # self.controller_list_topic_name = '/hsrb/controller_manager/list_controllers'
            # controller_list_topic_name = None
            # directions = {'up': 'x',
            #              'forward': 'z',
            #              'side': 'y'}

        elif self.robot_name == 'iai_donbot':
            self.wrench_topic_name = '/kms40_driver/wrench'
            # arm_topic_name = '/scaled_pos_joint_traj_controller/command'
            # directions = {'up': 'y',
            #              'forward': 'z',
            #              'side': 'x'}
        else:
            rospy.logerr(f'{self.robot_name} is not supported')

    def __get_rospy_data(self,
                         data_compensated: WrenchStamped):
        if self.init_data:
            self.init_data = False
            self.prev_values = [data_compensated] * (self.order + 1)
            self.whole_data = {self.unfiltered: [data_compensated],
                               self.filtered: [data_compensated]}

        filtered_data = self.__filter_data(data_compensated)

        self.whole_data[self.unfiltered].append(data_compensated)
        self.whole_data[self.filtered].append(filtered_data)

        self.prev_values.append(data_compensated)
        self.prev_values.pop(0)

        if self.debug:
            print(
                f'x: {data_compensated.wrench.force.x}, '
                f'y: {data_compensated.wrench.force.y}, '
                f'z: {data_compensated.wrench.force.z}')

    def __get_filter(self, order=4, cutoff=10, fs=60):
        if self.filter_config == FilterConfig.butterworth:
            return Butterworth(order=order, cutoff=cutoff, fs=fs)

    def __filter_data(self, current_wrench_data: WrenchStamped) -> WrenchStamped:
        filtered_data = WrenchStamped()
        for attr in ['x', 'y', 'z']:
            force_values = [getattr(val.wrench.force, attr) for val in self.prev_values] + [
                getattr(current_wrench_data.wrench.force, attr)]
            torque_values = [getattr(val.wrench.torque, attr) for val in self.prev_values] + [
                getattr(current_wrench_data.wrench.torque, attr)]

            filtered_force = self.filter.filter(force_values)[-1]
            filtered_torque = self.filter.filter(torque_values)[-1]

            setattr(filtered_data.wrench.force, attr, filtered_force)
            setattr(filtered_data.wrench.torque, attr, filtered_torque)

        return filtered_data

    def subscribe(self):
        """
        Subscribe to the specified wrench topic.

        This will automatically be called on setup.
        Only use this if you already unsubscribed before.
        """

        self.force_torque_subscriber = rospy.Subscriber(name=self.wrench_topic_name,
                                                        data_class=WrenchStamped,
                                                        callback=self.__get_rospy_data)

    def unsubscribe(self):
        """
        Unsubscribe from the specified topic
        """
        self.force_torque_subscriber.unregister()

    def get_last_value(self, is_filtered=True) -> WrenchStamped:
        """
        Get the most current data values.

        :param is_filtered: Decides about using filtered or raw data

        :return: A list containing the most current values (newest are first)
        """
        status = self.filtered if is_filtered else self.unfiltered
        return self.whole_data[status][-1]

    def get_derivative(self, is_filtered=True) -> WrenchStamped:
        """
        Calculate the derivative of current data.

        :param is_filtered: Decides about using filtered or raw data
        """
        status = self.filtered if is_filtered else self.unfiltered

        before: WrenchStamped = self.whole_data[status][-2]
        after: WrenchStamped = self.whole_data[status][-1]
        derivative = WrenchStamped()

        derivative.wrench.force.x = before.wrench.force.x - after.wrench.force.x
        derivative.wrench.force.y = before.wrench.force.y - after.wrench.force.y
        derivative.wrench.force.z = before.wrench.force.z - after.wrench.force.z
        derivative.wrench.torque.x = before.wrench.torque.x - after.wrench.torque.x
        derivative.wrench.torque.y = before.wrench.torque.y - after.wrench.torque.y
        derivative.wrench.torque.z = before.wrench.torque.z - after.wrench.torque.z

        return derivative
