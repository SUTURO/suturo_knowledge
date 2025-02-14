#!/usr/bin/env python3
import rospy
from knowledge_msgs.srv import SaveInfo
from suturo_knowledge.interf_q import InterfacePlanningKnowledge

# When the Service "SaveInfo" gets called and receives
# an input, the function "save_person_data" of the interface "InterfacePlanningKnowledge" 
# is called.
# Input: "{id: '2.0', name: 'toni', drink: 'cola', interest: 'tennis', profession: 'doctor'}" 
# Output: returns a simple confirmation that the saving was complete

def save_this(info):

    rospy.loginfo("third method is called.")
    inter = InterfacePlanningKnowledge()

    result = inter.save_person_data(info.id, info.name, info.drink, info.interest, info.profession)
    rospy.loginfo(result)


if __name__ == '__main__':
    rospy.init_node('save_service_server')
    rospy.Service('save_server', SaveInfo, save_this)
    rospy.loginfo("save_server")
    rospy.spin()
