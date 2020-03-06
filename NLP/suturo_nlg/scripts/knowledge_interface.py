#!/usr/bin/env python
import rospy
import rosprolog_client

prolog = rosprolog_client.Prolog()


def what_is_on(surface):
    rospy.wait_for_service('/rosprolog/query')

    if surface == "shelf":
        query_string = "once((all_objects_on_shelves(_Objs), member(Obj,_Objs)))"
    elif surface == "big table":
        query_string = "once((rdf_urdf_name(Table, table_3_center),rdf_has(Obj, hsr_objects:'supportedBy', Table)))"
        solutions = prolog.all_solutions(query_string)

    solution = solutions[0]['Obj'].split('#')[1].split('_')[0]
    print(solution)
    return solution







if __name__ == '__main__':
    rospy.init_node('knowledge_interface')
    what_is_on()