#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import rosprolog_client

prolog = rosprolog_client.Prolog()


def callback(String):
    query = "where_to('Kitchen'" + "," + "PoseStamped)."
    rospy.loginfo(query)

    po = prolog.once(query)
    rospy.loginfo(po)

    pose_stamped_msg = PoseStamped()
    pose_stamped_msg.header.frame_id = po['PoseStamped'][0]

    pose_stamped_msg.pose.position.x = po['PoseStamped'][1][0]
    pose_stamped_msg.pose.position.y = po['PoseStamped'][1][1]
    pose_stamped_msg.pose.position.z = po['PoseStamped'][1][2]

    pose_stamped_msg.pose.orientation.x = po['PoseStamped'][2][0]
    pose_stamped_msg.pose.orientation.y = po['PoseStamped'][2][1]
    pose_stamped_msg.pose.orientation.z = po['PoseStamped'][2][2]
    pose_stamped_msg.pose.orientation.w = po['PoseStamped'][2][3]

    pub.publish(pose_stamped_msg)


if __name__ == '__main__':
    try:
        rospy.init_node('whatever')
        # Sub
        rospy.Subscriber('whatever', String, callback)
        # Pub
        pub = rospy.Publisher('/poseStamped', PoseStamped, queue_size=30)
        rospy.loginfo('PoseStamped_determined')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
