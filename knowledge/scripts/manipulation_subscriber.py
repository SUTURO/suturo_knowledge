#!/usr/bin/env python
import rospy
from manipulation_msgs.msg import ObjectInGripper
import rosprolog_client

prolog = rosprolog_client.Prolog()

def callback(data):

    if data.mode == 0:
        object_frame_id = data.object_frame_id
        query = "once(attach_object_to_gripper(hsr_objects:'" + object_frame_id + "'))."
        rospy.loginfo(query)
        prolog.all_solutions(query)
    elif data.mode == 1:
        position = data.goal_pose.pose.position
        posx = position.x
        posy = position.y
        posz = position.z

        quaternion = data.goal_pose.pose.orientation
        quatx = quaternion.x
        quaty = quaternion.y
        quatz = quaternion.z
        quatw = quaternion.w
        query = "once(release_object_from_gripper([[" + str(posx) + "," + str(posy) + "," + str(posz) + "],[" + \
                str(quatx) + "," + str(quaty) + "," + str(quatz) + "," + str(quatw) + "]]))."

        rospy.loginfo(query)
        prolog.all_solutions(query)



def listener():
    rospy.init_node('manipulation_subscriber', anonymous=True)

    rospy.Subscriber("object_in_gripper", ObjectInGripper, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
