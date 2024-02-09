#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose

import tf2_ros
import tf2_geometry_msgs

# Is this object fragile?

import rospy
import rosprolog_client
prolog = rosprolog_client.Prolog()
from knowledge_msgs.srv import IsFragile

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

        
def get_pose(name):

    #de_obj = "'http://www.ease-crc.org/ont/SOMA.owl#Cup'"
    #query = "object_pose("+ de_obj + ", PoseStamped)."
    #query = "object_pose(suturo:'Bowl', PoseStamped)."

    dings = crop(name)
    print(dings)

    if str(dings) == "Table_VRIKLDCZ":

        query = "position_tableVRIKLDCZ(X)."
        print(query)

        sol = prolog.once(query)
        print(sol)

    return True


def transform_pose(table_name):

    

    # welcher table + frame
    #
    #if table_name == "popcorn table":
    #    com1 = "iai_kitchen/popcorn_table:table:table_center"

    #elif table_name == "kitchen counter":
    #    com2 = "iai_kitchen/table_kitchen_counter:table:table_center"

    #elif table_name == "long table":
    #    com3 = "iai_kitchen/long_table:table:table_center"

    # welche tables gibt es?
    query = "has_type(X, soma:'Table')."
    sol = prolog.all_solutions(query)
    print(sol)
    
    # wenn es tables gibt, frage nach pose stamped
    if len(sol) != 0:
        for table in sol:
            print(table)
            new_table = crop(table).replace('}', "")
            print(new_table)
            queryy = "object_pose("+ str(new_table) + ",X)."
            print(queryy)
            tpos = prolog.once(queryy)
            print(tpos)

            # vergleiche je, ob tisch name "table_name" und gesuchtes frame hat
            # true: transform
            # tpos = [old_frame, [0,0,0] [0,0,0,1]]
            table_frame = str(list(tpos.items())[0][1][0])
            print(table_name)
            print(table_frame)

            if table_name == "popcorn table" and table_frame == "iai_kitchen/popcorn_table:table:table_center":
                new_pose = transform_frame(tpos, "map", 1)
                return new_pose
            
            elif table_name == "kitchen counter" and table_frame == "iai_kitchen/table_kitchen_counter:table:table_center":
                new_pose = transform_frame(tpos, "map", 1)
                return new_pose
            
            elif table_name == "long table" and table_frame == "iai_kitchen/long_table:table:table_center":
                new_pose = transform_frame(tpos, "map", 1)
                return new_pose
            
            else: print("No right table")

    else: print("No solution tables")
    



def transform_frame(tpos, map_frame, n):

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    try:
        output_pose_stamped = tf_buffer.transform(tpos, map_frame, rospy.Duration(n))
        return output_pose_stamped.pose
                
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise



def crop(name):
    dpunkt_index = str(name).find(":")
    if dpunkt_index != -1:
        ex_string = str(name)[dpunkt_index +1:]
        de_string = ex_string.strip(' "')


        return de_string 

if __name__ == '__main__':
    rospy.init_node('fragility_service_server')
    rospy.Service('fragility_server', IsFragile, fragility_check)
    rospy.Service('fragility_serverr', IsFragile, get_pose)
    rospy.Service('test_tabel', IsFragile, transform_pose)
    rospy.loginfo("fragility_server")
    rospy.spin()
