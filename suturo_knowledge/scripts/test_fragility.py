#!/usr/bin/env python3

# Is this object fragile?

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from std_srvs.srv import IsFragile

def known_person(name):
    rospy.loginfo("test_fragility is called")
    newname = crop(name)
     
    query = "subclass_of(suturo:'"+newname+"', X), subclass_of(X, A),	triple(A, B, suturo:'Fragility')."
    
    #if newname == "CerealBowl":
    #    query = "has_propertyCerealBowl(X)."
    #elif newname == "MetalMug":
    #    query = "has_propertyMetalMug(X)."
    #query = "has_property(suturo:'"+newname+"')."
    rospy.loginfo(query)
    bool = prolog.once(query)
    rospy.loginfo(bool)
    if bool:
        return True
    else: 
        return False 



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
