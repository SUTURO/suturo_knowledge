#!/usr/bin/env python
from time import sleep

import rospy
import rosprolog_client
from knowledge_1920.msg import DetectedObject

prolog = rosprolog_client.Prolog()


def callback(data):
    rospy.wait_for_service('/rosprolog/query')
    source_frame = 'map'
    name = data.name
    depth = str(data.depth)
    width = str(data.width)
    height = str(data.height)
    r = str(data.color.r)
    g = str(data.color.g)
    b = str(data.color.b)
    a = str(data.color.a)
    x = str(data.pose.pose.position.x)
    y = str(data.pose.pose.position.y)
    z = str(data.pose.pose.position.z)
    qx = str(data.pose.pose.orientation.x)
    qy = str(data.pose.pose.orientation.y)
    qz = str(data.pose.pose.orientation.z)
    qw = str(data.pose.pose.orientation.w)
    threshold = "0.05"
    relativ_tf_tree = data.relativ_tf_tree

    # surface_query = 'table_surface(Surface)'
    surface_query = "rdf_equal(Surface, 'http://knowrob.org/kb/robocup.owl#kitchen_description_shelf_floor_%s_piece'),"
    obj_class = 'Other'  # 'Pringlespaprika'  # TODO This is temporary and should be changed

    query_string = (surface_query +
                    "create_object_at(hsr_objects:'" +
                    obj_class + "'," +
                    "['" + source_frame +
                    "', _, [" + ", ".join([x,y,z]) + "]," +
                    "[" + ", ".join([qx,qy,qz,qw]) + "]]," +
                    threshold + ", ObjectInstance," +
                    "[" + ", ".join([depth,width,height]) + "], " +
                    "[" + ", ".join([r,g,b,a]) + "])," +
                    "assert_object_on(ObjectInstance, Surface).")
    rospy.loginfo('Send query: \n' + query_string)
    solutions = prolog.all_solutions(query_string)
    rospy.loginfo(solutions)

    sleep(3)
    rospy.loginfo("Grouping objects.")
    prolog.all_solutions("group_shelf_objects.")


def listener():
    rospy.init_node('perception_listener', anonymous=False)
    rospy.Subscriber("/perception_output", DetectedObject, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
