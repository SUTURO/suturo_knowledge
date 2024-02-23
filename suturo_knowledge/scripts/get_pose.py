#!/usr/bin/env python3
import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from geometry_msgs.msg import PoseStamped
from knowledge_msgs.srv import ObjectPose


#################################################################################
## Object pose
        
def get_pose(table_name):
    # get all known tables
    q1 = "has_type(X, soma:'Table')."
    sol = prolog.all_solutions(q1)
    
    # for all tables, ask for their poses
    if len(sol) != 0:
        for table in list(sol):
            print("tables:" + str(table))
            new_table = crop(table).replace('}', "")
            q2 = "object_pose("+ str(new_table) + ",X)."
            tposs = prolog.once(q2)

            # extract table name and table frame 
            table_frame = str(list(tposs.items())[0][1][0])
            print("1:" + str(table_name))
            print("2:" + str(table_frame))

            tn = crop(table_name)
            print("3:" + str(tn))
            
            # compare whether there is a table named "table_name" and has searched frame
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
        return None 

###########################################################################
## return information about table as PoseStamped
    
def get_table_pose(new_table):
    new_pose = "object_pose(" + str(new_table) + ", [map, X, Y])."
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
## get object pose that depends on certain object property


def where_at(name):
    obj_name = crop(name)
    q1 = "what_object(" + "\'" + obj_name.lower() + "\'" + ", Object)."
    sol = prolog.once(q1)

    if len(sol) != 0:
        q2 = "is_perishable("+ "\'" + obj_name + "\')."
        soll = prolog.once(q2)
        
        if soll == dict():
            return get_pose("furniture:popcorn table")
        
        else: 
            print("Object exists but not perishable. You can find it on the kitchen counter!")
            return get_pose("furniture:kitchen counter")

    else:
        print("No object found under this name!")
        #return get_pose("furniture:long table")
        return None


#################################################################################
# crop magic
def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')
        return de_string

if __name__ == '__main__':
    rospy.init_node('pose_server')
    rospy.Service('pose_server', ObjectPose, get_pose)
    rospy.Service('where_to_find_server', ObjectPose, where_at)
    rospy.loginfo("pose_server, where_to_find_server")
    rospy.spin()