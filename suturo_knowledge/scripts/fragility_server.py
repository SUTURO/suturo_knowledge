#!/usr/bin/env python3

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from knowledge_msgs.srv import IsFragile

#######################################################################################################
## Fragility

def fragility_check(name):
    newname = crop(name)
    q1 = "what_object("+ "\'"+newname.lower()+ "\'" + ", Object)."
    sol = prolog.once(q1)
    print(sol)
    
    if len(sol) == 0:
        print("Sorry, object is not known to us!")
        return False
    
    else: 
        q2 = "fragility_new("+ "\'" + newname.lower() + "\')."
        soll = prolog.once(q2)

        if soll == dict():
            print("Object is fragile!")
            return True
        
        else: 
            print("Object exists but not fragile!")
            return False


#################################################################################
## crop magic
        
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
