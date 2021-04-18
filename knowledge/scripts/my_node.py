import rospy

if __name__ == '__main__':
    pub = rospy.Publisher('my_topic')
    rospy.init_node('my_node')
    while not rospy.is_shut_down():
        rospy.loginfo('loginfo')
        pub.publish('hello from my_node')
