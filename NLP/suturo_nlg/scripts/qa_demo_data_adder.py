#!/usr/bin/env python
import rospy
import rosprolog_client

prolog = rosprolog_client.Prolog()

rospy.init_node('qa_system_demo_data')

qs = "make_all_tables_source."
prolog.all_solutions(qs)
print("Made tables to source surfaces.")

qs = "make_all_shelves_target."
prolog.all_solutions(qs)
print("Made shelves to target surfaces.")

qs = "hsr_lookup_transform('map','enviorment/world',X,Y)."
solutions = prolog.all_solutions(qs)
print("Fixed? tf")


qs = "create_object_at(hsr_objects:'Pringlesoriginal', [map, _, [2.2,0.9,0.6], [0,0,0,1]], 0.05, ObjectInstance,[0.07,0.07,0.3],[255, 0, 0, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)
print("Added Pringlesoriginal to small table.")

qs = "create_object_at(hsr_objects:'Pringlessaltvinegar',[map, _, [-1.75,1.1,0.8],[0,0,0,1]],0.05, ObjectInstance,[0.07,0.07,0.3],[0, 0, 255, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)
print("Added PringlesSaltVinegar to big table.")

qs = "create_object_at(hsr_objects:'Cup',[map, _, [0.59,-0.1,0.85],[0,0,0,1]],0.05, ObjectInstance,[0.1,0.1,0.1],[0, 255, 0, 255]), place_object(ObjectInstance)."
solutions = prolog.all_solutions(qs)
print("Added cup to shelf.")

