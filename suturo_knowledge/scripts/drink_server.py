#!/usr/bin/env python3

# What is the favourite drink of person X?

import rospy
from knowledge_msgs.srv import GiveMeFavDrink
from suturo_knowledge.interf_q import InterfacePersonAndFavDrink


# When the Service "GiveMeFavDrink" gets called and receives
# a string with a name, the function "what_is_your_fav_drink" of the
# interface "InterfacePersonAndFavDrink" is called.
# Output: returns the (type of the) favourite drink of the person.

# Input: name: "Bob"
# Output: fav_drink:'http://www.ease-crc.org/ont/SUTURO.owl#Milk'

def fav_drink_of_person_x(Name):

    rospy.loginfo("First interface is called.")
    inter = InterfacePersonAndFavDrink()

    result = inter.what_is_your_fav_drink(Name)
    rospy.loginfo(result)
    return str(result)

  
if __name__ == '__main__':
    rospy.init_node('drink_service_server')
    rospy.Service('drink_server', GiveMeFavDrink, fav_drink_of_person_x)
    rospy.loginfo("drink_server")
    rospy.spin()
