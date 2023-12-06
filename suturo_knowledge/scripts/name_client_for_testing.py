#!/usr/bin/env python3

import rospy
from std_srvs.srv import NewService


def name_client(String):
    rospy.wait_for_service('name_service')
    try:
        serv = rospy.ServiceProxy('name_service', NewService)
        res = serv("Bob")
        return res.response_message
    
    except rospy.ServiceException as e:
        rospy.loginfo("Service call failed:", e)

if __name__ == '__main__':
    try:
        rospy.init_node('name_service_client')
        re = name_client("Bob")
        rospy.loginfo("Antwort:", re)
        
    except rospy.ROSInterruptException:
        pass

