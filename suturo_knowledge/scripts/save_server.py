#!/usr/bin/env python3

# Save name X and favourite drink Y

import rospy
from std_srvs.srv import ThreeService
from suturo_knowledge.interf_q import InterfaceSavePersonAndDrink


# When the Service "ThreeService" gets called and receives
# a string intent, string name and a string drink, the function "save_this" of the
# interface "InterfaceSavePersonAndDrink" is called.
# Input: 
# (1)'intent': SaveMe
# (2)'entities': {('NaturalPerson', 'X')}
#
# Output: 
# info with the saved name and drink

def save_this(Name, Drink):

    rospy.loginfo("third method is called :)")
    inter = InterfaceSavePersonAndDrink()

    result = inter.save_person_and_drink(Name, Drink)
    rospy.loginfo(result)
    return result


if __name__ == '__main__':
    rospy.init_node('save_service_server')
    rospy.Service('save_server', ThreeService, save_this)
    rospy.loginfo("save_server 1/4")
    rospy.spin()
