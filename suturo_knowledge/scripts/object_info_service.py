#!/usr/bin/env python3

import rospy
import json
from knowledge_msgs.srv import ObjectInfo


def get_object_info(object_name):
    rospy.wait_for_service('obj_info_server')  # Warte, bis der Service verfügbar ist
    try:
        # Erstelle einen Proxy für den Service
        obj_info_service = rospy.ServiceProxy('obj_info_server', ObjectInfo)

        # Rufe den Service mit dem Objektname auf
        response = obj_info_service(object_name)
        print("res:" + str(response))

        # Gib die Antwort zurück
        with open("output.json", "w") as json_file:
            json_file.write(str(response))

        return response
    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")


if __name__ == "__main__":
    rospy.init_node('obj_info_client')  # Initialisiere den Knoten

    # Beispielaufruf des Service
    while True:
        object_name = input("obj name: ")
        rospy.loginfo(f"Requesting info for object: {object_name}")
        result = get_object_info(object_name)

        if result:
            rospy.loginfo(f"Object Info: {result}")
        else:
            rospy.logerr("Failed to retrieve object information.")
  