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
    # rospy.loginfo("test")
    for marker in data.markers:
        # rospy.loginfo(marker)
        osa_message = ObjectStateArray()
        if marker.action == 0:
            osa_message.action = 0
        else:
            osa_message.action = 1
        os = ObjectState()
        os.object_id = marker.ns

        x = marker.pose.position.x
        y = marker.pose.position.y
        z = marker.pose.position.z

        # query = "hsr_lookup_transform('map',[" + str(x) + "," + str(y) + "," + str(z) + "], PoseTrans, _), hsr_existing_object_at_thr(PoseTrans, 0.009, Instance)."

        # query = "hsr_existing_object_at_thr([" + str(x) + "," + str(y) + "," + str(z) + "], 0.009, Instance)."

        # solutions = prolog_client.all_solutions(query)

        object_id = "http://www.semanticweb.org/suturo/ontologies/2020/3/objects#" + marker.header.frame_id # "http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Other_SJQWTXBE"
        os.frame_name = marker.header.frame_id # needs to be "Other_SJQWTXBE"
        os.object_type  = marker.header.frame_id.split('_')[0] # Other
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
    pub = rospy.Publisher('/object_state', ObjectStateArray, queue_size=10)
    
    rospy.loginfo('Waiting for rosprolog')    
    rospy.wait_for_service('/rosprolog/query')
    rospy.loginfo('Marker_to_ObjectStatePublisher Ready')    
    rospy.spin()
