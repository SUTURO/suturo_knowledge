#!/usr/bin/env python
import rospy
from suturo_nlg.msg import KeyValuePair, MeaningRepresentation


def talker():
    pub = rospy.Publisher('kinterface_in', MeaningRepresentation, queue_size=10)
    rospy.init_node('knowledge_interface_test', anonymous=True)

    mr1 = MeaningRepresentation()

    kvp1 = KeyValuePair()
    kvp1.key = "predicate"
    kvp1.value = "what is on"
    kvp2 = KeyValuePair()
    kvp2.key = "location"
    kvp2.value = "table"

    mr1.role_values.append(kvp1)
    mr1.role_values.append(kvp2)

    pub.publish(mr1)


    mr2 = MeaningRepresentation()

    kvp1.key = "predicate"
    kvp1.value = "is there"

    kvp2.key = "location"
    kvp2.value = "table"

    kvp3 = KeyValuePair()
    kvp3.key = "item"
    kvp3.value = "Pringles"

    mr2.role_values.append(kvp1)
    mr2.role_values.append(kvp2)
    mr2.role_values.append(kvp3)

    pub.publish(mr2)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass