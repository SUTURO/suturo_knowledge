#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import unittest
import rosunit
import random
import time
import rosprolog_client

#from knowledge_msgs.srv import ObjectInfo
#from knowledge_msgs.srv import IsFragile
#from knowledge_msgs.srv import SaveInfo
#from knowledge_msgs.srv import IsKnown

from suturo_knowledge.interf_q import InterfacePlanningKnowledge 
prolog = rosprolog_client.Prolog()
inter = InterfacePlanningKnowledge()

class TestGPSRQueries(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        """
        Initialize ROS node
        """
        rospy.init_node("GPSR_query_tests", anonymous=True)

###################################################################################################
### 1: is_fragile 

    def test_is_fragile(self):
        q1 = prolog.once(f"is_fragile(bowl).")
        print("Q1:", [q1]) # = {} ??
        self.assertIsNotNone(q1)
        #self.assertTrue(q1)

        q2 = prolog.once(f"is_fragile(chair).")
        #self.assertFalse(q2)

## function
        sol = self.assertFalse(inter.is_fragile('bed'))
        soll = self.assertTrue(inter.is_fragile('bowl'))

#- assertFalse und True mal gucken 
###################################################################################################
### 2: is_perishable

    def test_is_perishable(self):
        q1 = prolog.once(f"is_perishable(milk).")
        self.assertIsNotNone(q1)
        #self.assertTrue(q1)

        q2 = prolog.once(f"is_perishable(chair).")
        #self.assertIsNone(q2)
        #self.assertEquals(q2, {})
        #self.assertFalse(q2)

## function
        self.assertTrue(inter.is_perishable('cola'))
        self.assertFalse(inter.is_perishable('chair'))

#- assertFalse und True mal gucken 
###################################################################################################
### 3: grasp_pose

    def test_grasp_pose(self):
        sol = prolog.once(f"grasp_pose(spoon, Pose).")
        self.assertEquals(sol["Pose"], 'top')
        self.assertNotEquals(sol["Pose"], 'side')

## function 
        q1 = inter.grasp_pose('bowl')
        self.assertIsNotNone(q1)
        self.assertEquals(q1.strip(' "') , "'top'")

        q2 = inter.grasp_pose('knife')
        self.assertIsNotNone(q2)
        self.assertNotEquals(q2, 'side')

###################################################################################################
### 4: is_light_or_heavy

    def test_is_light_or_heavy(self):

        ## heavy
        q1 = prolog.once(f"is_light_or_heavy(bowl, Weight).")
        self.assertEquals(q1["Weight"], "heavy")

        ## light
        q2 = prolog.once(f"is_light_or_heavy(strawberry, Weight).")
        self.assertEquals(q2["Weight"], "light")

        ## x is light --> true
        q3 = prolog.once(f"is_light_or_heavy(bowl, heavy).")
        self.assertIsNotNone(q3)
        self.assertEquals(q3, {})

## function 
        q4 = inter.is_light_or_heavy('bowl', 'heavy')
        print(q4)
        self.assertIsNotNone(q4)
        self.assertTrue(q4)

        q5 = inter.is_light_or_heavy('strawberry', 'heavy')
        print(q5)
        self.assertFalse(q5)

# Ausgabe der Funktion tricky: bool 
# --> nur für Frage, ob Annahme true oder false
###################################################################################################
### 5: save_person_data

    def test_save_person_data(self):
        names = ["Anna", "Berta", "Connie", "Elena", "Felix", "Gabriel", "Hannes"]
        drinks = ["coffee", "tea", "water", "juice"]
        interests = ["reading", "cycling", "chess", "gaming"]
        professions = ["engineer", "teacher", "doctor", "artist"]
        
        name = random.choice(names)
        drink = random.choice(drinks)
        interest = random.choice(interests)
        profession = random.choice(professions)

        q1 = f"save_person_data(1.0, '{name}', {drink}, '{interest}', '{profession}')."
        sol = prolog.once(q1)
        self.assertEquals(sol, {})

## function 
        q2 = inter.save_person_data(1.0, name, drink, interest, profession)
        print(q2)
        self.assertTrue(q2)

## true, dass es funktioniert -> es kommt aber {} raus
###################################################################################################
### 6: call_person_info
# 6.1: calling info directly after saving

    def test_call_person_data_1(self):
        names = ["Anna", "Bert", "Connie", "Elena", "Felix", "Gabriel", "Hannes"]
        drinks = ["coffee", "tea", "water", "juice"]
        interests = ["reading", "cycling", "chess", "gaming"]
        professions = ["engineer", "teacher", "doctor", "artist"]

        name = random.choice(names)
        drink = random.choice(drinks)
        interest = random.choice(interests)
        profession = random.choice(professions)

        # save 
        q1 = prolog.once(f"save_person_data(1.0, '{name}', {drink}, '{interest}', '{profession}').")
        self.assertIsNotNone(q1)
        self.assertEquals(q1, {})

        # call
        q2 = prolog.once(f"call_person_data(1.0, Name, Drink, Interest, Profession).")
        qdrink = prolog.once(f"what_object('{drink}',Object).")

        self.assertIsNotNone(q2)
        self.assertEquals(q2["Name"], f"{name}")
        self.assertEquals(q2["Drink"], qdrink["Object"])
        self.assertEquals(q2["Interest"], f"{interest}")
        self.assertEquals(q2["Profession"], f"{profession}")

# 6.2: calling info after making changes

    def test_call_person_data_2(self):
        names = ["Anna", "Bert", "Connie", "Elena", "Felix", "Gabriel", "Hannes"]
        drinks = ["coffee", "tea", "water", "juice"]
        interests = ["reading", "cycling", "chess", "gaming"]
        professions = ["engineer", "teacher", "doctor", "artist"]

        name = random.choice(names)
        drink = random.choice(drinks)
        interest = random.choice(interests)
        interest2 = random.choice(interests)
        profession = random.choice(professions)
        profession2 = random.choice(profession)

        ## save data
        q1 = prolog.once(f"save_person_data(2.0, '{name}', {drink}, '{interest}', '{profession}').")
        self.assertIsNotNone(q1)

        ## change some data
        q2 = prolog.once(f"save_person_data(2.0, '{name}', {drink}, '{interest2}', '{profession2}').")
        self.assertIsNotNone(q2)
        
        ## call for new data
        q3 = prolog.once(f"call_person_data(2.0, Name, Drink, Interest, Profession).")
        self.assertEquals(q3["Interest"], f"{interest2}")
        self.assertEquals(q3["Profession"], f"{profession2}")

# 6.3: calling data with only one key

    def test_call_person_data_3(self):
        names = ["Anna", "Bert", "Connie", "Elena", "Felix", "Gabriela", "Hannes"]
        drinks = ["coffee", "tea", "water", "juice"]
        interests = ["reading", "cycling", "chess", "gaming"]
        professions = ["engineer", "teacher", "doctor", "artist"]

        name, name2 = random.sample(names, 2)
        drink = random.choice(drinks)
        drink2 = random.choice(drinks)
        interest = random.choice(interests)
        interest2 = random.choice(interests)
        profession = random.choice(professions)
        profession2 = random.choice(profession)

        ## person 1
        q1 = prolog.once(f"save_person_data(3.0, '{name}', {drink}, '{interest}', '{profession}').")
        self.assertIsNotNone(q1)

        ## person 2
        q2 = prolog.once(f"save_person_data(4.0, '{name2}', {drink2}, '{interest2}', '{profession2}').")
        self.assertIsNotNone(q2)

        ## person 2 by id 
        q3 = prolog.once(f"call_person_data(4.0, Name, Drink, Interest, Profession).")
        self.assertEquals(q3["Name"], f"{name2}")
        self.assertEquals(q3["Interest"], f"{interest2}")
        self.assertEquals(q3["Profession"], f"{profession2}")

###################################################################################################
### 7: has_predefined_location

    def test_predefined_location(self):
        # drink => shelf
        q1 = prolog.once(f"has_predefined_location('fanta', Location).")
        q2 = prolog.once(f"what_object('shelf', Loc).")
        self.assertEquals(q1["Location"], q2["Loc"])

        # cutlery => dishwasher 
        q3 = prolog.once(f"has_predefined_location('fork', Location).")
        q4 = prolog.once(f"what_object('dishwasher', Loc).")
        self.assertEquals(q3["Location"], q4["Loc"])

###################################################################################################
### 8: has_likely_location
# tested on GermanOpen map 
# fruits --> billy shelf
# cutlery --> on the dishwasher

    def test_has_likely_location(self):
        # Location = Shelf for fruit
        fruits = ["apple", "banana", "strawberry", "peach", "plum", "orange"]
        fruit = random.choice(fruits)

        q1 = prolog.once(f"has_likely_location({fruit}, Location, LocObj, Pose).")
        qs = prolog.once(f"what_object('shelf', Loc).")
        self.assertEquals(q1["Location"], 'Billy Shelf')


        # Location = Dishwasher for cutlery
        cutleries = ["knife", "fork", "spoon"]
        cutlery = random.choice(cutleries)

        q1 = prolog.once(f"has_likely_location({cutlery}, Location, LocObj, Pose).")
        qs = prolog.once(f"what_object('dishwasher', Loc).")
        self.assertEquals(q1["Location"], qs["Loc"])

###################################################################################################
### 9: has_likely_location_in_room
# tested on GermanOpen map

    def test_has_likely_location_in_room(self):
        # fruits
        fruits = ["apple", "banana", "strawberry", "peach", "plum", "orange"]
        cutleries = ["knife", "fork", "spoon"]
        room = ["kitchen", "living room"]

        fruit = random.choice(fruits)
        cutlery = random.choice(cutleries)

        # Room = kitchen
        q1 = prolog.once(f"is_kitchen(K), has_likely_location_in_room({cutlery}, K, LocObj, Pose).")
        #self.assertEqual()

        # Room = living room 
        q1 = prolog.once(f"is_living_room(L), has_likely_location_in_room({cutlery}, L, LocObj, Pose).")
        #self.assert()

###################################################################################################
### 10: navigability
# tested on GermanOpen map

    def test_navigability(self):
        room = ["kitchen", "living_room", "bedroom", "hallway", "office"]

        q1 = prolog.once(f"is_kitchen(K), navigability(K, Nav).")
        self.assertEquals(q1["Nav"], 1)

        q2 = prolog.once(f"is_living_room(L), navigability(L, Nav).")
        self.assertEquals(q2["Nav"], 2)

        q3 = prolog.once(f"is_hallway(H), navigability(H, Nav).")
        self.assertEquals(q3["Nav"], 0)

# output is a number
###################################################################################################
### 11: obj_characteristics

###################################################################################################
### 12: object_perceive_pose
# tested on GermanOpen map

    def test_object_perceive_pose(self):
        q1 = prolog.once(f"is_table(T), object_perceive_pose(T, _, PoseStamped).")
        checkPose = ['iai_kitchen/couch_table:couch_table:table_center', [0.845, 0.0, 0.0], [0, 0, 1, 0]]
        self.assertEquals(q1["PoseStamped"], checkPose)

###################################################################################################
### 13: init_gpsr_2024

    def test_init_gpsr_2024(self):
        q1 = prolog.once(f"init_gpsr_2024.")
        self.assertIsNotNone(q1)

#--> gibt ne Fehlermeldung im anderen Terminal; weil map gebraucht??
####################################################################################################
### 14: get_obj_instance_of_type
# tested on GermanOpen map

    def test_get_obj_instance_of_type(self):
        q1 = prolog.once(f"is_kitchen(K), has_type(K, Type).")
        q2 = prolog.once(f"what_object('kitchen', Obj).")
        self.assertEquals(q1["Type"], q2["Obj"])

###################################################################################################
### 15: get_room_entry_pose_class

    def test_get_room_entry_pose_class(self):
        q1 = prolog.once(f"findall(Room, is_room(Room), Rooms), map_entry_pose_on_rooms(Rooms).")
        self.assertIsNotNone(q1)

        q2 = prolog.once(f"is_kitchen(K), entry_pose(K, PoseStamped).")
        self.assertIsNotNone(q2)

###################################################################################################
### 16: get_room_pose
# tested on GermanOpen map 

    def test_get_room_pose(self):
        # entry
        q1 = prolog.once(f"is_kitchen(K), entry_pose(K, PoseStamped).")
        checkPose1 = ['map', [7.71, 1.35, -0.05], [0.0, 0.0, 0.0, 1.0]]
        self.assertEquals(q1["PoseStamped"], checkPose1)

        #exit 
        q3 = prolog.once(f"is_kitchen(K), exit_pose(K, PoseStamped).")
        checkPose2 = ['map', [7.66, 2.5, -0.05], [0.0, 0.0, 0.0, 1.0]]
        print(checkPose2)
        self.assertEquals(q3["PoseStamped"], checkPose2)


#- man soll wählen können, ob entry oder exit 
#- prolog.all_solutions ? 
###################################################################################################
### 17: get_all_room_poses 
# tested on GeranOpen map 

    def test_get_all_room_poses(self):
        # entries
        q1 = prolog.all_solutions(f"is_kitchen(K), entry_pose(K, PoseStamped).")
        self.assertIsNotNone(q1)

        # exits
        q2 = prolog.all_solutions(f"is_kitchen(K), exit_pose(K, PoseStamped).")
        self.assertIsNotNone(q2)

###################################################################################################
### 18: get_room_middle_pose
    # q1 = prolog.once(f"middle(Room, PoseStamped).")

###################################################################################################
### 19: get_nav_poses_for_furniture_item
# tested on GermanOpen_bringup map

    def test_get_nav_poses_for_furniture_item(self):

        q1 = prolog.once(f"what_object('couch table', Obj), has_type(ObjInst, Obj), what_object('living room', Room),  has_type(RoomInst, Room), is_inside_of(ObjInst, RoomInst), furniture_rel_pose(ObjInst, 'perceive', Pose).")
        checkPose = [['iai_kitchen/couch_table:couch_table:table_center', [-0.875, 0.0, -0.35], [0.0, 0.0, 0.0, 1.0]]]
        self.assertEquals(q1["Pose"], checkPose)

###################################################################################################
### 20: check_existence_of_instance

    def test_check_existence_of_instance(self):
        nlp_name = 'couch table'
        q1 = prolog.once(f"(what_object_transitive('couch table', Obj), instance_of(Inst, Obj)); (has_robocup_name(Obj, 'couch table')).")
        # just check first solution
        q2 = prolog.once(f"what_object('couch table', Obj).")
        #self.assertEquals(q1["Obj"], q2["Obj"])

# nlp name variabel machen
# kann sein, dass failed, denn es kann Inst oder Obj rauskommen 
###################################################################################################
### 21: check_existence_of_class

    def test_check_existence_of_class(self):
        q1 = prolog.once(f"what_object_transitive('table', Class).")
        self.assertIsNotNone(q1["Class"])
 
# nlp name variabel machen
###################################################################################################
### 22: get_predefined_source_item_location_name
# tested on GermanOpen map
##!!!!! FEHLER

    def test_get_predefined_source_item_location_name(self):
        #q0 = prolog.once(f"init_gpsr_2024.")
        q1 = prolog.once(f"init_gpsr_2024, what_object_transitive('cornflakes', Obj), predefined_origin_location(Obj, Furniture), furniture_rel_pose(Furniture, 'perceive', Pose).")
        checkPose = [['iai_kitchen/kitchen_island_block:kitchen_counter:table_center', [-1.2, 0.0, -1.0], [0.0, 0.0, 0.0, 1.0]]]
        self.assertEquals(q1["Pose"], checkPose)

# predefined_origin_location geht nur für gewisse dinge 
# predefined_origin_location(Obj, Furniture).
# subclass_of(X, 'http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFood').
###################################################################################################
### 23: get_predefined_source_item_location_iri
# tested on GermanOpen map
##!!!!! FEHLER

    def test_get_predefined_source_item_location_iri(self):
        #q0 = prolog.once(f"init_gpsr_2024.")
        q1 = prolog.once(f"init_gpsr_2024, create_object(Object, suturo:'Strawberry', ['map', [1,1,1], [0,0,0,1]]), has_type(Object, Type), predefined_origin_location(Type, Furniture), furniture_rel_pose(Furniture, 'perceive', Pose).")
        checkPose = [['iai_kitchen/couch_table:couch_table:table_center', [-0.875, 0.0, -0.35], [0.0, 0.0, 0.0, 1.0]]]
        self.assertEquals(q1['Pose'], checkPose)

# scheinbar gibt es Probleme mit "is" in furniture_rel_pose wenn table = dishwasher table
###################################################################################################
### 24: get_predefined_destination_item_location
# tested on GermanOpen map
##!!!!! FEHLER

    def test_get_predefined_destination_item_location(self):
        #q0 = prolog.once(f"init_gpsr_2024.")
        q1 = prolog.once(f"init_gpsr_2024,create_object(Object, suturo:'Apple', ['map', [1,0,1], [0,0,0,1]]), has_type(Object, Type), predefined_destination_location(Type, Furniture), furniture_rel_pose(Furniture, 'perceive', Pose).")
        checkPose =[['iai_kitchen/couch_table:couch_table:table_center', [-0.875, 0.0, -0.35], [0.0, 0.0, 0.0, 1.0]]]
        self.assertEquals(q1['Pose'], checkPose)


# habe noch has_type ergänzt, ansonsten funkt queries nicht
# falsch = what_object_transitive(Obj_XYZ, Name)
# richtig = what_object_transitive('juice', Name)
###################################################################################################
## 25: check_existence_based_on_class
    # q1 = prolog.once(f"what_object_transitive(Name, {class_iri}).")

# 
###################################################################################################


if __name__ == '__main__':
    rosunit.unitrun(
        'suturo_knowledge',      # package name
        'GPSR_query_tests',      # test name
        TestGPSRQueries          # TestCase class
    )