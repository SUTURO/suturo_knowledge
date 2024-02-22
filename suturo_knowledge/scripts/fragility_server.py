#!/usr/bin/env python3

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from knowledge_msgs.srv import IsFragile

#######################################################################################################
## Fragility

def fragility(name):
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


def fragility_check(name):
    newname = crop(name)
    rospy.loginfo(newname)

    query = "what_object("+ "\'"+newname.lower()+ "\'" + ", Object)."
    rospy.loginfo(query)
    sol = prolog.once(query)
    rospy.loginfo(sol)
    
    if len(sol) == 0:
        print("Sorry, object is not known to us!")
        return False
    else: 
        queryy = "fragility_new("+ "\'" + newname.lower() + "\')."
        print(queryy)

        soll = prolog.once(queryy)
        if soll == dict():
            return True
        else: 
            print("Object: yes; not fragile")
            return False


#################################################################################
# crop magic
def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')
        return de_string 
    


if __name__ == '__main__':
    rospy.init_node('fragility_server')
    rospy.Service('fragility_server', IsFragile, fragility_check)
    rospy.loginfo("fragility_server")
    rospy.spin()
