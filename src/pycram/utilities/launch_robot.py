import time

import roslaunch
import rospy
import rospkg


def launch_pr2():
    # name = 'pr2'
    # urdf = 'pr2.urdf'
    executable = 'pr2_standalone.launch'
    launch_robot(executable)


def launch_hsrb():
    # name = 'hsrb'
    # urdf = 'hsrb.urdf'
    executable = 'hsrb_standalone.launch'
    launch_robot(executable)


def launch_armar6():
    # name = 'armar6'
    # urdf = 'armar6.urdf'
    executable = 'armar6_standalone.launch'

    launch_robot(executable)


def launch_robot(launch_file, package='pycram', launch_folder='/launch/'):
    rospath = rospkg.RosPack()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    launch = roslaunch.parent.ROSLaunchParent(uuid, [rospath.get_path(package) + launch_folder + launch_file])
    launch.start()

    rospy.loginfo(f'{launch_file} started')

    # Wait for ik server
    time.sleep(2)
