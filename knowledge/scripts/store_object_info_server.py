#! /usr/bin/env python

import rospy
import rosprolog_client
import actionlib

import knowledge_msgs.msg as action

prolog = rosprolog_client.Prolog()


# This Script is an Actionserver that takes the Action knowledge_msgs/StoreObjectInfo.action
# The sent data will one by one be stored in the knowledgebase. Unless this process is being
# preempted or fails, this Server will return succeeded=true.
class StoreObjectInfoServer(object):
    _feedback = action.StoreObjectInfoFeedback()
    _result = action.StoreObjectInfoResult()

    def __init__(self, name):
        self._action_name = name
        self._as = actionlib.SimpleActionServer(self._action_name, action.StoreObjectInfoAction,
                                                execute_cb=self.execute_cb, auto_start=False)
        self._as.start()

    def execute_cb(self, goal):
        rospy.loginfo("knowledge store server got called")

        success = True

        for data in goal.detectionData:
            if self._as.is_preempt_requested():
                rospy.loginfo('%s: Preempted' % self._action_name)
                self._as.set_preempted()
                success = False
                break

            rospy.wait_for_service('/rosprolog/query')

            obj_class = str(data.obj_class)
            if obj_class:
                obj_class = obj_class.capitalize().replace('_', '')
            else:
                rospy.loginfo("The given class name is empty. Setting to OTHER.")
                obj_class = "Other"

            class_test_query = "is_class(hsr_objects:'" + obj_class + "')." # via knowrob, eg. Gummy Bears / Pringles Originals
            print(class_test_query)
            solutions = prolog.all_solutions(class_test_query) # bool
            if not solutions:  # if class is not known
                rospy.logwarn(
                    "The class '" + obj_class + "' has no equivalent in kowledge-ontology. Setting class to Other.")
                obj_class = "Other" # <- instantiate as Other
            # --- further information from perception
            confidence_class = str(data.confidence_class)
            shape = str(data.shape)
            source_frame = 'map'
            depth = str(data.depth)
            width = str(data.width)
            height = str(data.height)
            r = str(data.color.r)
            g = str(data.color.g)
            b = str(data.color.b)
            a = str(data.color.a)
            confidence_color = str(data.confidence_color)
            x = str(data.pose.pose.position.x)
            y = str(data.pose.pose.position.y)
            z = str(data.pose.pose.position.z)
            qx = str(data.pose.pose.orientation.x)
            qy = str(data.pose.pose.orientation.y)
            qz = str(data.pose.pose.orientation.z)
            qw = str(data.pose.pose.orientation.w)
            region_splits = str(data.region).split('_')

            # checks if the obj position is valid, eg. distance to surface
            # (1) is_legal_obj_position
            # (2) create_object_at()
            query_string = ("is_legal_obj_position([" + ", ".join([x, y, z]) + "]),create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#" + # actually create_object(...)
                                obj_class + "'," + confidence_class + ", " +
                                "['" + source_frame +
                                "', _, [" + ", ".join([x, y, z]) + "]," +
                                "[" + ", ".join([qx, qy, qz, qw]) + "]], _," +
                                "[" + ", ".join([depth, width, height]) + "], " + shape + ", _, " +
                                "[" + ", ".join([r, g, b]) + "], " + confidence_color + ",_).")
            rospy.loginfo('Send query: \n' + query_string)
            solutions = prolog.all_solutions(query_string)
            if not solutions:
                rospy.logwarn("This Object couldn't have been added: " + data.obj_class)
            
            # when we identified an object, and see another object next to it, then:
            #   create a concept as group: eg. salty snacks!
            #   -> over the object hierarchy: go upwards and find
            #   -> lowest common parent
            # maybe remove, else: (...)
            rospy.loginfo("Grouping objects.")
            prolog.all_solutions("group_objects_at([" + ", ".join([x,y,z]) + "]).")

        self._result.succeeded = success # the Action.result
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('store_object_info_server')
    server = StoreObjectInfoServer("store_object_info_server")
    rospy.spin()
