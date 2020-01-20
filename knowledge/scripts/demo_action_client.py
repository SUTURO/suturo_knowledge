#! /usr/bin/env python
from __future__ import print_function

import rospy

import actionlib

import suturo_perception_msgs.msg as message
import knowledge_msgs.msg as action

_data = None


# this script is designed to bypass planning in a demo.
# It directly takes the perception-output postet at perception_actionserver/result
# and calls the StoreObjectInfo Action with that data.
def test_perception():
    rospy.Subscriber("perception_actionserver/result", message.ExtractObjectInfoActionResult, callback)

    while _data is None:
        i = 1
        rospy.loginfo("doin' nothing")
        # do nothing

    client = actionlib.SimpleActionClient('store_object_info_server', action.StoreObjectInfoAction)
    rospy.loginfo(str(_data))
    client.wait_for_server()
    goal = action.StoreObjectInfoGoal(detectionData=_data)

    client.send_goal(goal)

    client.wait_for_result()

    return client.get_result()


def callback(perceived_object_list):
    global _data
    _data = perceived_object_list.result.detectionData


if __name__ == '__main__':
    try:
        rospy.init_node('store_objects_action')

        result = test_perception()
        if result:
            rospy.loginfo("Actionserver returned success")
        else:
            rospy.loginfo("Actionserver returned failure")
        print("worked")
    except rospy.ROSInterruptException:
        print("program interrupted before completion", file=sys.stderr)
