#!/usr/bin/env python3

import rospy
import rosprolog_client


prolog = rosprolog_client.Prolog()

##################################################################
# 1: 
# What's person X's favourite drink?
# drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, Name):
        rospy.loginfo("First Interface is working")

        #crop the input string to an useful string
        crop_string = crop(Name)

        # query to ask Xs favourite drink
        query = "fav_drink(" + crop_string + "," + "X)."
        rospy.loginfo(query)
       
        solution = prolog.once(query)
        return solution

##################################################################
# 2:
# Is person X already known to us?
# name_server

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, name):
        rospy.loginfo("Second Interface is working")

        #crop the input string to an useful string
        crop_string = crop(name)
        
        query = "is_customer("+ crop_string +")."
        
        rospy.loginfo(query)
        solution = prolog.once(query)
        rospy.loginfo(solution)


        # if the Name is already known we don't want to save it again
        if solution == dict():
            rospy.loginfo("Your name is " + crop_string + "! Welcome back !!!")
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
            rospy.loginfo(test_call)
            rospy.loginfo("But now we know you!")

            return False

##################################################################
# 3:
# Save name X and favourite drink Y
# save_server

class InterfaceSavePersonAndDrink:

    def save_person_and_drink(self, info):

        rospy.loginfo(str(info))

        # info is a string of the form name_drink: "name, drink"
        # we want to extract the name
        
        # first split where the comma is
        parts = str(info).split(", ")
        
        # remove unnecessary chars such as ":" and apostrophes
        name_part = parts[0].split(": ")[1].strip(' "').lower() if len(parts) > 0 else ""
        drink_part = parts[1].strip(' "') if len(parts) > 1 else ""


        print("Name:", name_part)
        print("Drink:", drink_part)
    
        rospy.loginfo("Third Interface is working")
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
        return "Your name is " + name_part + " and your favourite drink is " + drink_part + "." + " We saved your information!"

#crop magic
def crop(String):
    colon_index = str(String).find(":")
    if colon_index != -1:
        ex_string = str(String)[colon_index +1:]
        print("ex_string:" + ex_string)

        de_string = ex_string.strip(' "')
        print("de_string:" + de_string)
        return de_string.lower()
