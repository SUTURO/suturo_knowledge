#!/usr/bin/env python
# license removed for brevity
import rospy
from knowledge_1920.msg import DetectedObject


def talker():
    pub = rospy.Publisher('/perception_output', DetectedObject, queue_size=10)
    rospy.init_node('dummyPerceptionPublisher', anonymous=False)
    rate = rospy.Rate(1)  # 1hz
    id = 1
    while not rospy.is_shutdown():
        id = id +1
        # publish this on /perception_output

        msgData = DetectedObject()


        msgData.name = 'Object:' + str(id)
        msgData.confidence = 0.9
        msgData.shape = 1
        # geometry_msgs / PoseStamped
        msgData.pose.header.seq = id
        msgData.pose.header.stamp = rospy.get_rostime()
        msgData.pose.header.frame_id = 'map'

        msgData.pose.pose.position.x = 0.0
        msgData.pose.pose.position.y = 0.0
        msgData.pose.pose.position.z = 0.0

        msgData.pose.pose.orientation.x = 0.0
        msgData.pose.pose.orientation.y = 0.0
        msgData.pose.pose.orientation.z = -0.0123938397363
        msgData.pose.pose.orientation.w = 0.999923196466

        msgData.width = 0.05  # size meters
        msgData.height = 0.05
        msgData.depth = 0.05
        msgData.color.r = 200
        msgData.color.b = 1
        msgData.color.g = 1
        msgData.color.a = 0

        msgData.relativ_tf_tree  = 'map'  # use 'map' for now

        pub.publish(msgData)
        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass