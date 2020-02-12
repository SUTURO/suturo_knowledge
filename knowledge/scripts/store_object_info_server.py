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
            surface_query = ""
            if str(data.region).endswith("table"):
                surface_query = 'table_surface(Surface),'
            elif "shelf" in region_splits:
                rospy.loginfo("Asserting object on shelf floors " + str(region_splits))
                surface_query = "rdf_equal(Surface, 'http://knowrob.org/kb/robocup.owl#kitchen_description_shelf_floor_%s_piece')," \
                                % region_splits.pop()
            else:
                rospy.loginfo("Issue with region: %s" % data.region)

            # rospy.loginfo(prolog.all_solutions(surface_query + " true."))

            # filter_plane_noise_query = "{}" \
            #                            "surface_pose_in_map(Surface, [[_,_,Z],_]), " \
            #                            "ZOffset is {} - Z," \
            #                            "ZOffset > 0.015.".format(surface_query, z)
            #
            # plane_solutions_raw = prolog.all_solutions(filter_plane_noise_query)
            # # print(plane_solutions_raw)
            valid = True
            # if not plane_solutions_raw:
            #     rospy.loginfo("Invalid Z-pose of the object. Would be under or in the surface.")
            #     valid = False
            # # if not volume < 8.0:
            # #     rospy.loginfo("Volume: {} is too high to be a valid object.".format(volume))
            # #     valid = False
            # if float(data.depth) > 0.3 or float(data.width) > 0.3 or float(data.height) > 0.3:
            #     rospy.loginfo("One of the dimensions is bigger than 30 cm.")
            #     valid = False
            if valid:
                # rospy.loginfo("Object is at valid height and of valid volume")
                query_string = (surface_query +
                                "create_object_at(hsr_objects:'" +
                                obj_class + "'," +
                                "['" + source_frame +
                                "', _, [" + ", ".join([x, y, z]) + "]," +
                                "[" + ", ".join([qx, qy, qz, qw]) + "]]," +
                                threshold + ", ObjectInstance," +
                                "[" + ", ".join([depth, width, height]) + "], " +
                                "[" + ", ".join([r, g, b, a]) + "])," +
                                "assert_object_on(ObjectInstance, Surface).")
                rospy.loginfo('Send query: \n' + query_string)
                solutions = prolog.all_solutions(query_string)
                rospy.loginfo(solutions)
                if not solutions:
                    rospy.logwarn("This Object couldn't have been added: " + obj_class)
            else:
                rospy.loginfo("IGNORING object of class: " + obj_class)

        rospy.loginfo("Grouping objects.")
        prolog.all_solutions("group_shelf_objects.")

        self._result.succeeded = success
        self._as.set_succeeded(self._result)


if __name__ == '__main__':
    rospy.init_node('store_object_info_server')
    server = StoreObjectInfoServer("store_object_info_server")
    rospy.spin()
