#!/usr/bin/env python3

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from knowledge_msgs.srv import ObjectInfo
from suturo_knowledge.interf_q import InterfacePlanningKnowledge

# Deliver informations about object: type, heavy, fragile

# When the Service "GiveMeFavDrink" gets called and receives
# a string with a name, the function "what_is_your_fav_drink" of the
# interface "InterfacePersonAndFavDrink" is called.
# Output: returns the (type of the) favourite drink of the person.

# Input: name: "Bob"
# Output: fav_drink:'http://www.ease-crc.org/ont/SUTURO.owl#Milk'

#def fav_drink_of_person_x(Name):

#    rospy.loginfo("First interface is called.")
#    inter = InterfacePlanningKnowledge()

    #result = inter.what_is_your_fav_drink(Name)
#    result = inter.place_pose_object(Name)
#    rospy.loginfo(result)
#    return str(result)

###############################################################################
# Input: object name: milk
# Output: obj info: (Milk, type: Drink, color: "Yellow", fragile: No, Heavy: No, ...)
def get_obj_info(Object):
    inter = InterfacePlanningKnowledge()
    rospy.loginfo("Interface is called successfully")

    return inter.obj_characteristics(Object)
    print(result)
    #return str(result)
  
if __name__ == '__main__':
    rospy.init_node('obj_info_server')
    rospy.Service('obj_info_server', ObjectInfo, get_obj_info)
    rospy.loginfo("obj_info_server")
    rospy.spin()
