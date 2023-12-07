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

    def do_we_known_u(self, name):

        # weil der übergebene param als name:"X" dargestellt wird, 
        #   müssen wir erst nur X extrahieren 
        colon_index = str(name).find(":")
        if colon_index != -1:
            ex_string = str(name)[colon_index +1:]
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
