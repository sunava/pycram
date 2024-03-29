import rospy
from std_msgs.msg import String
from demos.pycram_receptionist_demo.utils.new_misc import *
response = ""
callback = False

# Declare variables for humans
host = HumanDescription("Bob Ross", fav_drink="art")
guest1 = HumanDescription("guest1")
guest2 = HumanDescription("guest2")
seat_number = 2

pub_nlp = rospy.Publisher('/startListener', String, queue_size=10)


def data_cb(data):
    global response
    global callback

    response = data.data.split(",")
    callback = True


def misc_fct():
    global response
    global callback

    rospy.Subscriber("/nlp_out", String, data_cb)

    print("Hello, i am Toya and my favorite drink is oil. What about you, talk to me?")
    rospy.sleep(1)

    # signal to start listening
    pub_nlp.publish("start listening")

    while not callback:
        rospy.sleep(1)
        print("wait")
    callback = False

    if response[0] == "<GUEST>":
        if response[1].strip() != "None":
            print("it is so noisy here, please confirm if i got your name right")
            guest1.set_drink(response[2])
            rospy.sleep(2)
            guest1.set_name(name_confirm(response[1]))

        else:
            # save heard drink
            guest1.set_drink(response[2])

            # ask for name again once
            guest1.set_name(name_repeat())

        # confirm favorite drink
        guest1.set_drink(drink_confirm(guest1.fav_drink))

    else:
        # two chances to get name and drink
        i = 0
        while i < 2:
            print("please repeat your name and drink loud and clear")
            pub_nlp.publish("start")

            while not callback:
                rospy.sleep(1)
            callback = False

            if response[0] == "<GUEST>":
                guest1.set_name(response[1])
                guest1.set_drink(response[2])
                break
            else:
                i += 1

    # stop looking
    print("i will show you the living room now")
    print("hello " + str(guest1.name) + " your favorite drink is " + str(guest1.fav_drink))

    print("end")


misc_fct()
