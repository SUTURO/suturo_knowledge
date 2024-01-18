#!/usr/bin/env python3

# Is this object fragile?

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from std_srvs.srv import IsFragile
#from suturo_knowledge.interf_q import InterfaceDoWeKnowYou

# When the Service "IsKnown" gets called and receives
# a string with a name, the function "known_person" of the
# interface "InterfaceDoWeKnowYou" is called.
# Output: returns a bool: true if known, else false

# Input: name: "Bob"
# Output: is_known: true

def known_person(name):
    rospy.loginfo("test_fragility is called")
    newname = crop(name)
    query = "subclass_of(suturo:'"+newname+"', X), subclass_of(X, A),	triple(A, B, suturo:'Fragility')."
    bool = prolog.once(query)
    rospy.loginfo(bool)
    if bool:
        return True
    else: return False 



def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')

        return de_string

if __name__ == '__main__':
    rospy.init_node('fragility_service_server')
    rospy.Service('fragility_server', IsFragile, known_person)
    rospy.loginfo("fragility_server")
    rospy.spin()
