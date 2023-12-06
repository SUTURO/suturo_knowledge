#!/usr/bin/env python3

# Is person X already known to us?

import rospy
from std_srvs.srv import TwoService
from suturo_knowledge.interf_q import InterfaceDoWeKnowYou
#from suturo_knowledge.interf_q import Test

# When the Service "TwoService" gets called and receives
# a string intent and a string name, the function "known_person" of the
# interface "InterfaceDoWeKnowYou" is called.
# Input: 
# (1)'intent': IsKnown
# (2)'entities': {('NaturalPerson', 'X')}
#
# Output: returns bool 
#         true = person is known
#         false = person is not known 

# 1. extract ('NaturalPerson', 'Bob')
# 2. call function with tuple

# def call_test(input_string):
#    inter = Test()
#    result = inter.hope_this_works(input_string)
#    return result

def known_person(Name):
    rospy.loginfo("second method is called :)")
    inter = InterfaceDoWeKnowYou()

    bool = inter.do_we_known_u(Name)
    rospy.loginfo(bool)
    return bool



if __name__ == '__main__':
    rospy.init_node('name_service_server')
    rospy.Service('name_server', TwoService, known_person)
    rospy.loginfo("name_server 2/4")
    rospy.spin()
