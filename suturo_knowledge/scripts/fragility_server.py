#!/usr/bin/env python3

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from knowledge_msgs.srv import IsFragile
from geometry_msgs.msg import PoseStamped
from knowledge_msgs.srv import ObjectPose

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
## Object pose
        
def get_pose(table_name):
   
    # all known tables
    query = "has_type(X, soma:'Table')."#, object_pose(X, [F, A, B])."
    sol = prolog.all_solutions(query)
    print(sol)
    liste = list(sol)
    print("liste:" + str(liste))
    # for all tables, ask for their poses
    if len(sol) != 0:
        for table in list(sol):
            print("tables:" + str(table))
            new_table = crop(table).replace('}', "")
            print(new_table)
            queryy = "object_pose("+ str(new_table) + ",X)."
            print(queryy)
            tposs = prolog.once(queryy)
            print(tposs)

            # compare whether there is table named "table_name" and has searched frame
            table_frame = str(list(tposs.items())[0][1][0])
            print("1:" + str(table_name))
            print("2:" + str(table_frame))

            tn = crop(table_name)
            print("3:" + str(tn))
            
            if tn == "popcorn table" and table_frame == "iai_kitchen/popcorn_table:table:table_center":
                return get_table_pose(new_table)
                
            elif tn == "kitchen counter" and table_frame == "iai_kitchen/table_kitchen_counter:table:table_center":
                return get_table_pose(new_table)
                 
            elif tn == "long table" and table_frame == "iai_kitchen/long_table:table:table_center":
                return get_table_pose(new_table)
            
            else:
                print("No right table")
                
    else: 
        print("No solution tables")
        return False

###########################################################################
## get pose 
## format
    
def get_table_pose(new_table):
    new_pose = "object_pose(" + str(new_table) + ", [map, X, Y])."
    print(new_pose)
    sol = prolog.once(new_pose)
    print("4:" + str(sol))
    
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position.x = list(sol.items())[0][1][0]
    pose_stamped.pose.position.y = list(sol.items())[0][1][1]
    pose_stamped.pose.position.z = list(sol.items())[0][1][2]

    pose_stamped.pose.orientation.x = list(sol.items())[1][1][0]
    pose_stamped.pose.orientation.y = list(sol.items())[1][1][1]
    pose_stamped.pose.orientation.z = list(sol.items())[1][1][2]
    pose_stamped.pose.orientation.w = list(sol.items())[1][1][3]

    return pose_stamped

#################################################################################
# crop magic
def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')
        return de_string 
    


if __name__ == '__main__':
    rospy.init_node('fragility_service_server')
    rospy.Service('fragility_server', IsFragile, fragility_check)
    rospy.Service('test_table', ObjectPose, get_pose)
    rospy.loginfo("fragility_server 4/4")
    rospy.spin()
