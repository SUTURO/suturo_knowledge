#!/usr/bin/env python3

import rospy
import rosprolog_client


prolog = rosprolog_client.Prolog()


#class Test:
#
#    def hope_this_works(self, input_string):
#        return "Received:" + str(input_string)


##################################################################
# 1: 
# What's person X's favourite drink?
# drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, Name):
        rospy.loginfo("First Interface is working")
        c_index = str(Name).find(":")
        if c_index != -1:
            ex_string = str(Name)[c_index +1:]
            print("ex_string:" + ex_string)

            de_string = ex_string.strip(' "')
            print("de_string:" + de_string)

        rospy.loginfo("First Interface is working")

        # query to ask Xs favourite drink
        query = "fav_drink(" + de_string + "," + "X)."
        rospy.loginfo(query)
       
        solution = prolog.once(query)
        return solution

##################################################################
# 2:
# Is person X already known to us?
# name_server

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, Name):

        # weil der übergebene param als name:"X" dargestellt wird, 
        #   müssen wir erst nur X extrahieren 
        colon_index = str(Name).find(":")
        if colon_index != -1:
            ex_string = str(Name)[colon_index +1:]
            print("ex_string:" + ex_string)

            de_string = ex_string.strip(' "')
            print("de_string:" + de_string)


        rospy.loginfo("Second Interface is working")
        
        query = "is_customer("+ de_string +")."
        
        rospy.loginfo(str(query))
        solution = prolog.once(query)
        rospy.loginfo(solution)

        if solution:
            rospy.loginfo("Your name is " + de_string + "! Welcome back !!!")
            return True
        else:
            save = "save_me("+ de_string +")."
            rospy.loginfo(save)
            save_call = prolog.once(save)
            rospy.loginfo(str(save_call))
            rospy.loginfo("We saved you!")

            test = "is_customer("+de_string+")."
            rospy.loginfo(test)
            test_call = prolog.once(test)
            rospy.loginfo(str(test_call))
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
        name_part = parts[0].split(": ")[1].strip(' "') if len(parts) > 0 else ""
        drink_part = parts[1].strip(' "') if len(parts) > 1 else ""


        print("Name:", name_part)
        print("Drink:", drink_part)
    
        rospy.loginfo("Third Interface is working")


        query = "save_me_and_drink(" + name_part + "," + drink_part + ")."
        prolog.once(query)
        
        # return confitmation
        return "Your name is " + name_part + " and your favourite drink is " + drink_part + "." + " We saved your information!"

