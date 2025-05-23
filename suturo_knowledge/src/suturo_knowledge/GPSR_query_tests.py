import rospy
import random
import unittest
import rosunit
import rosprolog_client
from knowledge_msgs.srv import ObjectInfo
from knowledge_msgs.srv import IsFragile
from knowledge_msgs.srv import SaveInfo
from knowledge_msgs.srv import IsKnown
from suturo_knowledge.interf_q import InterfacePlanningKnowledge 

prolog = rosprolog_client.Prolog()
inter = InterfacePlanningKnowledge()

class TestGPSRQueries(unittest.TestCase):

### 1/12: is_fragile 
## true
## false

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


### 2/12: is_perishable
## true
## false
    def test_is_perishable(self):
        q1 = prolog.once(f"is_perishable(milk).")
        self.assertIsNotNone(q1)
        #self.assertTrue(q1)

        q2 = prolog.once(f"is_perishable(chair).")
        #self.assertFalse(q2)

## function
        self.assertTrue(inter.is_perishable('cola'))
        self.assertFalse(inter.is_perishable('chair'))


### 3/12: grasp_pose
## top
## side

    def test_grasp_pose(self):
        sol = prolog.once(f"grasp_pose(spoon, Pose).")
        self.assertEquals(sol["Pose"], 'top')
        self.assertNotEquals(sol["Pose"], 'side')

## function 
        q1 = inter.grasp_pose('bowl')
        self.assertIsNotNone(q1)
        print("QQQQ1:", [q1])
        # strip did not work ???
        self.assertEquals(q1.strip(' "') , "'top'")

        q2 = inter.grasp_pose('knife')
        self.assertNotEquals(q2, 'side')


### 4/12: is_light_or_heavy
## light
## heavy
## x is light --> true

    def test_is_light_or_heavy(self):

        q1 = prolog.once(f"is_light_or_heavy(bowl, Weight).")
        self.assertEquals(q1["Weight"], "heavy")

        q2 = prolog.once(f"is_light_or_heavy(strawberry, Weight).")
        self.assertEquals(q2["Weight"], "light")

        q3 = prolog.once(f"is_light_or_heavy(bowl, heavy).")
        #self.assertTrue(q3)

##function
# Ausgabe der Funktion tricky: bool vs. light/heavy
# funktion erlaubt gerade nur Eingabe einer var
        #self.assertTrue(inter.is_light_or_heavy('bowl', 'heavy'))
        #self.assertFalse(inter.is_light_or_heavy('bowl', 'light'))

        q4 = inter.is_light_or_heavy('dishwasher tab')
        print("5Q4:", [q4])
        self.assertEquals(q4["Weight"], "light")


### 5.1/12: save_person_data
## true, dass es funktioniert -> es kommt aber {} raus
# true kommt bei funktion raus

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


###5.2: save_info_server
# noch unsicher, in welcher form die infos gegeben werden - einzeln, gebuendelt als string, dict? 
    #def test_save_info_server(self, info):

# ...

### 6/12: call_person_info
## !! call_person_data('1.0', 'A', milk, Hobby, Profession). = false = drink nicht var 
# ==> eingabe als 'milk', dann strip() zu milk, damit what_object 

# sollen dann die Funktionen dazu aus interf_q weg? oder auch einfach testen --> testen


# 1. calling data directly after saving
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
        q1 = f"save_person_data(1.0, '{name}', {drink}, '{interest}', '{profession}')."
        sol = prolog.once(q1)
        print("sol:", [sol])

        self.assertIsNotNone(sol)
        self.assertEquals(sol, {})

        # call
        q2 = f"call_person_data(1.0, Name, Drink, Interest, Profession)."
        soll = prolog.once(q2)
        print("SOLL:", [soll])
        qd = f"what_object('{drink}',Object)."
        print(qd)
        fdrink = prolog.once(qd)
        print(fdrink)

        self.assertIsNotNone(soll)
        self.assertEquals(soll["Name"], f"{name}")
        self.assertEquals(soll["Drink"], fdrink["Object"])
        self.assertEquals(soll["Interest"], f"{interest}")
        self.assertEquals(soll["Profession"], f"{profession}")

# 2. calling info after making changes
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
        q1 = f"save_person_data(2.0, '{name}', {drink}, '{interest}', '{profession}')."
        sol = prolog.once(q1)
        self.assertIsNotNone(sol)

        ## change some data
        q2 = f"save_person_data(2.0, '{name}', {drink}, '{interest2}', '{profession2}')."
        soll = prolog.once(q2)
        self.assertIsNotNone(sol)
        
        ## call for new data
        q3 = f"call_person_data(2.0, Name, Drink, Interest, Profession)."
        qsol = prolog.once(q3)

        self.assertEquals(qsol["Interest"], f"{interest2}")
        self.assertEquals(qsol["Profession"], f"{profession2}")

# 3. calling data with only one key
    def test_call_person_data_3(self):
        names = ["Anna", "Bert", "Connie", "Elena", "Felix", "Gabriel", "Hannes"]
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
        q1 = f"save_person_data(1.0, '{name}', {drink}, '{interest}', '{profession}')."
        sol = prolog.once(q1)
        self.assertIsNotNone(sol)

        ## person 2
        q2 = f"save_person_data(2.0, '{name2}', {drink2}, '{interest2}', '{profession2}')."
        sol = prolog.once(q2)
        self.assertIsNotNone(sol)

        qs1 = f"call_person_data(ID, '{name2}', Drink, Interest, Profession)."
        qsol1 = prolog.once(qs1)
        print("QSOL1:", [qsol1])
        self.assertEquals(qsol1["ID"], 2.0)

        self.assertEquals(qsol1["Interest"], f"{interest2}")
        self.assertEquals(qsol1["Profession"], f"{profession2}")


## function
#--> entnehme Elemente für die Abfrage, da gerade true mit variablen verglichen wird

        q1 = inter.call_person_data(1.0, name, drink, interest, profession)
        print("Q1:", [q1])
        #self.assertTrue(q1)



### 7/12: has_predefined_location

    def test_predefined_location(self):
        q1 = f"has_predefined_location('fanta', Location)."
        sol = prolog.once(q1)
        print("SOLL:", [sol])
        q2 = f"what_object('shelf', Loc)."
        qs = prolog.once(q2)
        self.assertEquals(sol["Location"], qs["Loc"])


### 8/12: has_likely_location
# fruits --> billy shelf
# cutlery --> on the dishwasher
# wie muss query aussehen generell ??
    def test_has_likely_location(self):
        # fruits
        fruits = [fruit, apple, banana, strawberry, peach, plum, orange]
        fruit = random.choice(fruits)

        # Location = Shelf for fruit
        q1 = prolog.once(f"has_likely_location({fruit}, Location, LocObj, Pose).")
        qs = prolog.once(f"what_object('shelf', Loc).")

        self.assertEquals(q1["Location"], qs["Loc"])


        # Location = Dishwasher for cutlery
        cutleries = [cutlery, knife, fork, spoon]
        cutlery = random.choice(cutleries)

        # Location = Shelf for fruit
        q1 = prolog.once(f"has_likely_location({cutlery}, Location, LocObj, Pose).")
        qs = prolog.once(f"what_object('dishwasher', Loc).")

        self.assertEquals(q1["Location"], qs["Loc"])



### 9/12: has_likely_location_in_room


### 10/12: navigability
# need map for testing that !!!
    def test_navigability(self):

        q1 = prolog.once(f"is_kitchen(K),"
                         f"navigability(K, Nav).")
        #self.assertEquals(q1["Nav"], Nav)

        q2 = prolog.once(f"is_bedroom(B),"
                         f"navigability(B, Nav).")
        #self.assertEquals(q2["Nav"], Nav)


##function - gibts noch nicht
        #q3 = inter.navigability()
        #print("Q3:", [q4])
        

### 11/12: object_characteristics
### 12/12: object_perceive_pose




if __name__ == '__main__':
    rosunit.unitrun(
        'knowrob_ros',           # package name
        'GPSR_query_tests',  # test name
        TestKnowrobRosLib        # TestCase class
    )