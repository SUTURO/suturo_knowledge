#! /usr/bin/env python

import rospy
import rosprolog_client
# marker
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
# object_state
from knowledge_msgs.msg import ObjectStateArray
from knowledge_msgs.msg import ObjectState

prolog_client = rosprolog_client.Prolog()

def callback(data):
    for marker in data.markers:
        osa_message = ObjectStateArray()
        if marker.action == 0:
            osa_message.action = 0
        else:
            osa_message.action = 1
        os = ObjectState()
        os.object_id = marker.ns

        x = marker.pose.point.x
        y = marker.pose.point.y
        z = marker.pose.point.z

        query = "hsr_lookup_transform('map'," + marker.header.frame_id + ", PoseTrans, _), hsr_existing_object_at_thr(PoseTrans, 0.009, Instance)."

        solutions = prolog_client.all_solutions(query)

        print(solutions)

        os.frame_name = solutions[0]['Instance']
        # os.object_type TODO do we need This??
        os.shape = 1  # TODO everything is a cube for now
        os.mesh_path = marker.mesh_resource  # will always be ""
        os.color = marker.color
        os.size = marker.scale
        os.pose.header = marker.header
        os.pose.pose = marker.pose
        # TODO, will leave this unbound os.static_transforms

        osa_message.object_states.append(os)
        pub.publish(osa_message)


if __name__ == '__main__':
    rospy.init_node('marker_to_objectstate')
    # subscriber
    rospy.Subscriber("/visualization_marker_array", MarkerArray, callback)
    # publisher
    pub = rospy.Publisher('/object_state_2', ObjectStateArray, queue_size=10)
    
    rospy.loginfo('Waiting for rosprolog')    
    rospy.wait_for_service('/rosprolog/query')
    rospy.loginfo('Marker_to_ObjectStatePublisher Ready')    
    rospy.spin()
