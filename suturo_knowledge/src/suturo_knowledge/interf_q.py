#!/usr/bin/env python3

import rospy
import rosprolog_client


prolog = rosprolog_client.Prolog()

#####################

class Test:
    def testing(self, string):
        rospy.loginfo("Testing on!")

        query = "cap_fst_letter(alina)"
        solution = prolog.once(query)

        rospy.loginfo(solution)

##################################################################
# 1: 
# What's person X's favourite drink?
# drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, name):
        rospy.loginfo("First Interface is working.")

        #crop the input string to an useful string
        crop_string = crop(name)

        # query to ask Xs favourite drink
        query = "fav_drink(" + crop_string + "," + "X)."
        rospy.loginfo(query)
       
        solution = prolog.once(query)
        rospy.loginfo(solution)

        crop_sol = crop_plus(solution)
        #reforms = "has_type(" + solution + "," + "X)."
        #ref = prolog.once(reforms)
        rospy.loginfo(crop_sol)
        # TODO: solution needs to be formated 
        return crop_sol

##################################################################
# 2:
# Is person X already known to us?
# name_server

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, name):
        rospy.loginfo("Second Interface is working.")

        #crop the input string to an useful string
        crop_string = crop(name)
        rospy.loginfo(crop_string)

        query = "is_customer("+ crop_string +")."
        rospy.loginfo(query)

        solution = prolog.once(query)
        rospy.loginfo("Should be empty first:" + str(solution))


        # if the Name is already known we don't want to save it again
        if solution == dict():
            rospy.loginfo("Your name is " + crop_string.capitalize() + "! Welcome back !!!")
            return True
        else:
            save = "save_me("+ crop_string +")."
            rospy.loginfo(save)

            save_call = prolog.once(save)
            rospy.loginfo(save_call)
            rospy.loginfo("We saved you!")

            test = "is_customer("+crop_string+")."
            rospy.loginfo(test)

            test_call = prolog.once(test)
            rospy.loginfo("Test saving:" + str(test_call))
            rospy.loginfo("But now we know you!")

            return False

##################################################################
# 3:
# Save name X and favourite drink Y
# save_server

class InterfaceSavePersonAndDrink:

    def save_person_and_drink(self, info):

        rospy.loginfo(info)

        # info is a string of the form name_drink: "name, drink"
        # we want to extract the name
        
        # first split where the comma is
        parts = str(info).split(", ")
        
        # remove unnecessary chars such as ":" and apostrophes
        name_part = parts[0].split(": ")[1].strip(' "').lower() if len(parts) > 0 else ""
        drink_part = parts[1].strip(' "') if len(parts) > 1 else ""


        print("Name:", name_part)
        print("Drink:", drink_part)
    
        rospy.loginfo("Third Interface is working.")
        if drink_part == "Coffee":
            query = "save_me_and_coffee(" + name_part +")."
            prolog.once(query)

        elif drink_part == "RaspberryJuice":
            query = "save_me_and_raspberryjuice(" + name_part +")."
            prolog.once(query)

        elif drink_part == "Milk":
            query = "save_me_and_milk(" + name_part +")."
            prolog.once(query)

        elif drink_part == "Tea":
            query = "save_me_and_tea(" + name_part +")."
            prolog.once(query)

        
        # return confirmation
        return "Your name is " + name_part.capitalize() + " and your favourite drink is " + drink_part + "." + " We saved your information!"

#crop magic
def crop(String):
    dpunkt_index = str(String).find(":")
    if dpunkt_index != -1:
        ex_string = str(String)[dpunkt_index +1:]
        print("ex_string:" + ex_string)

        de_string = ex_string.strip(' "')
        print("de_string:" + de_string)
        return de_string.lower()

def crop_plus(string):
    crop(string)

    # search '#'
    hi = str(string).find('#')
    # begin at '#'
    ui = str(string).find('_', hi)

    if hi != -1 and ui != -1:
        new_str = str(string)[hi + 1:ui]
        return new_str

    else:
        print("No '#' or no '_'!!")
