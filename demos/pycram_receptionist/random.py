import rospy
from std_msgs.msg import String
from std_srvs.srv import IsKnown

#from pycram.external_interfaces.knowrob import instances_of, get_guest_info
#import random

#def client_call(id):
#    rospy.init_node("client_node")
#    rospy.wait_for_service("serviceName")
#    rate = rospy.Rate(1)
#    while not rospy.s_shutdown():
#        try:
#            service_variable = rospy.ServiceProxy("SeviceName", Type)
#            respone = service_variable(parameterForService)
#            rate.sleep()
#        except rospy.ServiceExcepion as e:
#            print("Service call failed")


def get_guest_info(id):
    """
    function that uses Knowledge Service to get Name and drink from new guest via ID
    """

    #TODO: Service für Name und Getränk mit ID einbauen, wenn dieser fertig
    rospy.wait_for_service('name_server')
    print("debug")
    try:
        #TODO: Service class herausfinden
        print("in try")
        info_service = rospy.ServiceProxy('name_server', IsKnown)
        print("serverProxy")
        guest_data = info_service(id) #guest_data = List der Form ["name", "drink"]
        print("end")
        return guest_data
    except rospy.ServiceException as e:
        print("Service call failed")


if __name__ == '__main__':
   print("starte test")
   print(get_guest_info("4.0"))
