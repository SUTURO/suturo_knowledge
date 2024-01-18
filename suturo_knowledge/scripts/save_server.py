#!/usr/bin/env python3

# Save name X and favourite drink Y

import rospy
from std_srvs.srv import SaveInfo
from suturo_knowledge.interf_q import InterfaceSavePersonAndDrink

# When the Service "SaveInfo" gets called and receives
# a string with a name and a string with a drink, the function 
# "save_person_and_drink" of the interface "InterfaceSavePersonAndDrink" 
# is called.
# Output: returns a simple confirmation that the saving was complete.

# Input: name: "Lisa, Milk, 3.0"
# Output: confirm: "Your name is Lisa and your favourite drink is Milk.
#                       We saved your Information!"

def save_this(Info):

    rospy.loginfo("third method is called.")
    inter = InterfaceSavePersonAndDrink()

    result = inter.save_person_and_drink(Info)
    rospy.loginfo(result)
    return result


if __name__ == '__main__':
    rospy.init_node('save_service_server')
    rospy.Service('save_server', SaveInfo, save_this)
    rospy.loginfo("save_server 1/3")
    rospy.spin()
