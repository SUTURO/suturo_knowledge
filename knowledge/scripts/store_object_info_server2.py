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
            if obj_class and float(data.confidence_class) > 0.5:
                obj_class = obj_class.capitalize().replace('_', '')
            else:
                rospy.loginfo("The given class name is empty. Setting to OTHER.")
                obj_class = "Other"

            class_test_query = "kb_is_class(hsr_objects:'" + obj_class + "')."
            solutions = prolog.all_solutions(class_test_query)
            if not solutions:
                rospy.logwarn(
                    "The class '" + obj_class + "' has no equivalent in kowledge-ontology. Setting class to Other.")
                obj_class = "Other"

            # confidence_class = '1.0' if data.confidence_class == 0.0 else data.confidence_class
            # shape = str(data.shape)
            source_frame = 'map'
            depth = str(data.depth)
            width = str(data.width)
            height = str(data.height)
            r = str(data.color.r)
            g = str(data.color.g)
            b = str(data.color.b)
            a = str(data.color.a)
            volume = float(data.depth) * float(data.width) * float(data.height) * 1000
            x = str(data.pose.pose.position.x)
            y = str(data.pose.pose.position.y)
            z = str(data.pose.pose.position.z)
            qx = str(data.pose.pose.orientation.x)
            qy = str(data.pose.pose.orientation.y)
            qz = str(data.pose.pose.orientation.z)
            qw = str(data.pose.pose.orientation.w)
            threshold = "0.05"
            region_splits = str(data.region).split('_')

            query_string = ("create_object_at(hsr_objects:'" +
                                obj_class + "'," +
                                "['" + source_frame +
                                "', _, [" + ", ".join([x, y, z]) + "]," +
                                "[" + ", ".join([qx, qy, qz, qw]) + "]]," +
                                threshold + ", ObjectInstance," +
                                "[" + ", ".join([depth, width, height]) + "], " +
                                "[" + ", ".join([r, g, b, a]) + "])," +
                            "object_supportable_by_surface(ObjectInstance, SurfaceLink)," +
                            "assert_object_on(ObjectInstance, SurfaceLink).")
            rospy.loginfo('Send query: \n' + query_string)
            solutions = prolog.all_solutions(query_string)
            rospy.loginfo(solutions)
            if not solutions:
                rospy.logwarn("This Object couldn't have been added: " + obj_class)

        rospy.loginfo("Grouping objects.")
        prolog.all_solutions("group_shelf_objects.")

        self._result.succeeded = success
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('store_object_info_server')
    server = StoreObjectInfoServer("store_object_info_server")
    rospy.spin()