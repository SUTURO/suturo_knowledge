#!/usr/bin/env python3
import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from geometry_msgs.msg import PoseStamped
from knowledge_msgs.srv import ObjectPose


#################################################################################
## Object pose
"""       
def get_pose(table_name):
    # get all known tables
    q1 = "has_type(X, soma:'Table')."
    sol = prolog.all_solutions(q1)
    
    # for all tables, ask for their poses
    if len(sol) != 0:
        for table in list(sol):
            print("tables:" + str(table))
            new_table = crop2(table).replace('}', "")
            q2 = "object_pose("+ str(new_table) + ",X)."
            tpos = prolog.once(q2)

            # extract table name and table frame 
            table_frame = str(list(tpos.items())[0][1][0])
            print("1:" + str(table_name))
            print("2:" + str(table_frame))

            tn = crop2(table_name)
            print("3:" + str(tn))
            # verwende has_robocup_name in furniture_info.pl
            # compare whether there is a table named "table_name" and has searched frame
            if tn == "popcorn table" and table_frame == "iai_kitchen/popcorn_table:p_table:table_center":
                return get_table_pose(new_table)
                
            elif tn == "kitchen counter" and table_frame == "iai_kitchen/table_kitchen_counter:kc_table:table_center":
                return get_table_pose(new_table)
                 
            elif tn == "long table" and table_frame == "iai_kitchen/long_table:l_table:table_center":
                return get_table_pose(new_table)
            
            else:
                print("No right table")

    else: 
        print("No solution tables")
        return None 
###########################################################################
def get_pose_of_handle(handle_name):
    q1 = "has_type(X, soma:'DesignedHandle')."
    sol = prolog.all_solutions(q1)
    
    # for all handles, ask for their pose
    if len(sol) != 0:
        for handle in list(sol):
            print("handles:" + str(handle))
            new_handle = crop2(handle).replace('}', "")
            print("print new_handle:"+ new_handle)


            q2 = "object_pose("+ str(new_handle) + ",X)."
            print(q2)
            tpos = prolog.once(q2)
            handle_frame = str(list(tpos.items())[0][1][0])
            print("1:" + str(handle_name))
            print("2:" + str(handle_frame))

            tn = crop2(handle_name)
            print("3:" + str(tn))
            # verwende has_robocup_name in furniture_info.pl
            # compare whether there is a table named "table_name" and has searched frame
            if tn == "door_inside" and handle_frame == "iai_kitchen/iai_kitchen:arena:door_handle_inside":
                return get_table_pose(new_handle)
            elif tn == "door_outside" and handle_frame == "iai_kitchen/iai_kitchen:arena:door_handle_outside":
                return get_table_pose(new_handle)
            elif tn == "dishwasher" and handle_frame == "iai_kitchen/sink_area_dish_washer_door_handle":
                return get_table_pose(new_handle)
            elif tn == "shelf_left" and handle_frame == "iai_kitchen/shelf:shelf:shelf_door_left:handle":
                return get_table_pose(new_handle)
            elif tn == "shelf_right" and handle_frame == "iai_kitchen/shelf:shelf:shelf_door_right:handle":
                return get_table_pose(new_handle)
            else:
                print("No right handle")

    else: 
        print("No solution handles")
        return None 


###########################################################################
## return information about table as PoseStamped
    
def get_table_pose(new_table):
    new_pose = "object_pose(" + str(new_table) + ", [map, X, Y])."
    sol = prolog.once(new_pose)
    print("4:" + str(sol))
    
    pose_stamped = PoseStamped()
    pose_stamped.header.frame_id = "map"
    pose_stamped.pose.position.x = sol['X'][0]
    pose_stamped.pose.position.y = sol['X'][1]
    pose_stamped.pose.position.z = sol['X'][2]

    pose_stamped.pose.orientation.x = sol['Y'][0]
    pose_stamped.pose.orientation.y = sol['Y'][1]
    pose_stamped.pose.orientation.z = sol['Y'][2]
    pose_stamped.pose.orientation.w = sol['Y'][3]

    return pose_stamped

#################################################################################
## get object pose that depends on certain object property


def where_at(name):
    obj_name = crop2(name)
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


if __name__ == '__main__':
    rospy.init_node('pose_server')
    #rospy.Service('pose_server', ObjectPose, get_pose)
    rospy.Service('pose_server', ObjectPose, get_pose_of_handle)
    rospy.Service('where_to_find_server', ObjectPose, where_at)
    rospy.loginfo("pose_server, where_to_find_server")
    rospy.spin()

"""