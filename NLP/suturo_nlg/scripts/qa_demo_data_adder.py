#!/usr/bin/env python
import rospy
import rosprolog_client

prolog = rosprolog_client.Prolog()

rospy.init_node('qa_system_demo_data')

rospy.wait_for_service('/rosprolog/query')

qs = "make_all_tables_source."
prolog.all_solutions(qs)
print("Made tables to source surfaces.")

qs = "make_all_shelves_target."
prolog.all_solutions(qs)
print("Made shelves to target surfaces.")

qs = "create_object_at(hsr_objects:'Pringlesoriginal', [map, _, [-1.22,-2.6,0.8], [0,0,0,1]], 0.05, ObjectInstance,[0.1,0.1,0.3],[255, 0, 0, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)

qs = "create_object_at(hsr_objects:'Pringlessaltvinegar',[map, _, [0.4,4.7,1],[0,0,0,1]],0.05, ObjectInstance,[0.07,0.07,0.3],[0, 0, 255, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)

qs = "create_object_at(hsr_objects:'Oreos',[map, _, [1.3,-0.1,0.8],[0,0,0,1]],0.05, ObjectInstance,[0.2,0.2,0.2],[255, 255, 255, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)

qs = "create_object_at(hsr_objects:'Cup',[map, _, [1.1,-0.4,0.85],[0,0,0,1]],0.05, ObjectInstance,[0.1,0.1,0.1],[0, 255, 0, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)

qs = "create_object_at(hsr_objects:'Cup',[map, _, [-3,-5.5,1],[0,0,0,1]],0.05, ObjectInstance,[0.1,0.1,0.1],[0, 255, 0, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)
