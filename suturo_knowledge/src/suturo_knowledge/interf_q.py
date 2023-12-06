#!/usr/bin/env python3

import rospy
import rosprolog_client

prolog = rosprolog_client.Prolog()


#class Test:

#    def hope_this_works(self, input_string):
#        return "Received:" + str(input_string)


##################################################################
# 1: 
# What's person X's favourite drink?
# drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, Name):
        rospy.loginfo("First Interface is working")
        # query to ask Xs favourite drink
        query = "fav_drink(" + str(Name) + "," + "X)."
        # get all solutions
        all = prolog.once(query)
        # return all solutions (in a list enumerated)
        return all


##################################################################
# 2:
# Is person X already known to us?
# name_server

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, Name):
        rospy.loginfo("Second Interface is working")
        query = "is_known(" + Name + ")."
        solution = prolog.once(query)
    
        if solution:
            rospy.loginfo("Your name is "+ Name +"! Welcome back !!!")
            return solution
        else:
            save = "save_me("+ Name + ")."
            save_call = prolog.once(query)
            rospy.loginfo("Now we know you!")
            return solution

##################################################################
# 3:
# Save name X and favourite drink Y
# save_server

class InterfaceSavePersonAndDrink:

    def save_person_and_drink(self, Name, Drink):
        rospy.loginfo("Third Interface is working")
        query = "save_me_and_drink(" + Name + "," + Drink + ")."
        save_call = prolog.once(query)
        # return bestaetigung
        return "Your name is " + str(Name) + " and your favourite drink is " + str(Drink) + "."


##################################################################
# 4:
# (What's this persons name?)

class InterfaceWhatsYourName:

    def what_is_your_name(self, String):
        rospy.loginfo("Fourth Interface is working")

        query = ";"
        # get all solutions
        all = prolog.all_solutions(query)
        # return all solutions (in a list enumerated)
        return all


if __name__ == '__main__':
    rospy.init_node('interfaces_for_NLP', anonymous=True)
    rospy.loginfo("interf_q 4/4")
