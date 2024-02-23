#!/usr/bin/env python3

# Is person X already known to us?

import rospy
from knowledge_msgs.srv import IsKnown
#from suturo_knowledge.interf_q import InterfaceDoWeKnowYou
from suturo_knowledge.interf_q import InterfaceGivePersonID
from suturo_knowledge.interf_q import InterfacePersonAndFavDrink

# When the Service "IsKnown" gets called and receives
#   a string with an ID, in the method "person_id" the functions "whats_your_name" 
#   and "whats_your_fav_drink" are called.
# Output: returns "name, drink" or notice that there are no infos saved under this ID
"""
# When the Service "IsKnown" gets called and receives
# a string with a name, the function "known_person" of the
# interface "InterfaceDoWeKnowYou" is called.
# Output: returns a bool: true if known, else false

# Input: "Bob"
# Output: True or False

# ask if person is known and print if person has ID
# return bool

def known_person(name):
    inter = InterfaceDoWeKnowYou()
    inter2 = InterfaceGivePersonID()
    
    bool = inter.do_we_known_u(name)
    rospy.loginfo("Bool:" + str(bool))

    # which id does the known person have
    res = inter2.whats_your_id(name)
    rospy.loginfo(res)
    return bool
"""
# with the customer ID ask for information (favourite drink and name)
def person_info(guest_id):
    inter2_5 = InterfaceGivePersonID()
    inter3 = InterfacePersonAndFavDrink()

    # check for name saved under this ID
    gimme_name = inter2_5.whats_your_name(guest_id)

    if str(gimme_name) != "None" and gimme_name != []:
        
        # with name ask for favourite drink
        gimme_drink = inter3.what_is_your_fav_drink(str(gimme_name))

        # return "name", "drink"
        the_name = (crop(str(gimme_name)).replace("\'", "")).capitalize()
    
        rospy.loginfo(str(the_name) + ',' + gimme_drink)
        return str(the_name) + ',' + gimme_drink
    
    else:
        return "No name saved under this ID!"
        
    
# crop unnecessary chars and begin after the ':'
def crop(String):
    dpunkt_index = str(String).find(":")
    if dpunkt_index != -1:
        ex_string = str(String)[dpunkt_index +1:]
        print("ex_string:" + ex_string)
        de_string = ex_string.strip(' "').rstrip('}')
        print("de_string:" + de_string)
        return de_string
    

if __name__ == '__main__':
    rospy.init_node('name_service_server')
    #rospy.Service('name_server', IsKnown, known_person)
    rospy.Service('info_server', IsKnown, person_info)
    rospy.loginfo("info_server")
    rospy.spin()
