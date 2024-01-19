#!/usr/bin/env python3

# Is person X already known to us?

import rospy
from std_srvs.srv import IsKnown
from suturo_knowledge.interf_q import InterfaceDoWeKnowYou
from suturo_knowledge.interf_q import InterfaceGivePersonID
from suturo_knowledge.interf_q import InterfacePersonAndFavDrink

# When the Service "IsKnown" gets called and receives
# a string with a name, the function "known_person" of the
# interface "InterfaceDoWeKnowYou" is called.
# Output: returns a bool: true if known, else false

# Input: name: "Bob"
# Output: is_known: true


#Now: 
# Input: Name
# Output: ID 

def known_person(name):
    rospy.loginfo("second method is called")
    inter = InterfaceDoWeKnowYou()
    inter2 = InterfaceGivePersonID()
    
    bool = inter.do_we_known_u(name)
    rospy.loginfo("Bool:" + str(bool))
    #return bool

    # frage zu bekannter Person ihre ID
    res = inter2.whats_your_id(name)
    rospy.loginfo(res)

    return bool

def person_info(guest_id):
    rospy.loginfo("person_info - called")

    inter2_5 = InterfaceGivePersonID()
    inter3 = InterfacePersonAndFavDrink()

    # mit guest-ID frage: Was ist dein name?
    gimme_name = inter2_5.whats_your_name(guest_id)
    # bob
    # wenn es keine Namen gibt, erhält man None
    rospy.loginfo(gimme_name)

    if str(gimme_name) != "None" and gimme_name != []:
        
        # mit namen, frage: Was ist fav drink?
        gimme_drink = inter3.what_is_your_fav_drink(str(gimme_name))
        rospy.loginfo(gimme_drink)


        # return ["name", "drink"]
        the_name = (crop(str(gimme_name)).replace("\'", "")).capitalize()
        rospy.loginfo(the_name)
    
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
    rospy.Service('name_server', IsKnown, person_info)
    rospy.loginfo("name_server 2/3")
    rospy.spin()
