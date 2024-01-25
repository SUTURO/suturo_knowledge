#!/usr/bin/env python3

# Is this object fragile?

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from std_srvs.srv import IsFragile

def known_person(name):
    rospy.loginfo("test_fragility is called")
    newname = crop(name)
     
    query = "is_fragile("+newname+")."
    rospy.loginfo(query)
    bool = prolog.once(query)
    rospy.loginfo(bool)
    if bool == dict():
        return True
    else: 
        return False 

def test_call(name):
    newname = crop(name)
    rospy.loginfo(newname)

    query = "kb_call(holds(Name,suturo:hasPredefinedName,"+newname+")),is_fragile(Name)."
    rospy.loginfo(query)
    sol = prolog.once(query)
    rospy.loginfo(sol)
    if len(sol) == 0 :
        return False
    else: 
        return True


def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')

        return de_string

if __name__ == '__main__':
    rospy.init_node('fragility_service_server')
    rospy.Service('fragility_server', IsFragile, test_call)
    rospy.loginfo("fragility_server")
    rospy.spin()
