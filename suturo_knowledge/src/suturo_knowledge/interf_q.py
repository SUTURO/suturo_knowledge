#!/usr/bin/env python3

import rospy
import rosprolog_client

prolog = rosprolog_client.Prolog()

#########################################################################
# 1:
# Save name X and favourite drink Y
# used by: save_server

class InterfaceSavePersonAndDrink:

    def save_person_and_drink(self, info):
        rospy.loginfo("First Interface is working.")

        # info: name_drink: "name, drink"
          
        parts = str(info).split(", ")
        
        # remove unnecessary chars such as ":" and apostrophes
        name_part = parts[0].split(": ")[1].strip(' "').lower() if len(parts) > 0 else ""
        drink_part = parts[1].strip(' "') if len(parts) > 1 else ""

        print("Name:", name_part)
        print("Drink:", drink_part)
    

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

        else: 
            return ("sorry " + name_part.capitalize() + " but we don't know a drink named like "
                    + drink_part) 

        return ( "Your name is " + name_part.capitalize() 
                + " and your favourite drink is " + drink_part 
                + "." + " We saved your information!"
        )

#########################################################################
# 2:
# Is person X already known to us?
# used by: name_server

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, name):
        rospy.loginfo("Second Interface is working.")

        # crop the input string to a useful string
        crop_string = crop(name).lower()

        query = "is_customer("+ crop_string +")."
        rospy.loginfo(query)

        solution = prolog.once(query)
        rospy.loginfo("Should be empty first:" + str(solution))


        # save only when name is not already known
        if solution == dict():
            rospy.loginfo("Welcome back " + crop_string.capitalize() + "!")
            return True
        else:
            save = "save_me("+ crop_string +")."
            rospy.loginfo(save)

            save_call = prolog.once(save)
            rospy.loginfo(save_call)
            rospy.loginfo("We saved you!")
            rospy.loginfo("Nice to meet you" + crop_string.capitalize() + "!")

            test = "is_customer("+crop_string+")."
            rospy.loginfo(test)
            test_call = prolog.once(test)
            rospy.loginfo(test_call)

            # because person was previously not known
            return False

#########################################################################
# 3: 
# What's person X's favourite drink?
# used by: drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, name):
        rospy.loginfo("Third Interface is working.")

        # crop the input string to a useful string
        crop_string = crop(name).lower()
        rospy.loginfo("crop_string:" + crop_string)

        query = "fav_drink(" + crop_string + "," + "X)."
        rospy.loginfo(query)
       
        solution = prolog.once(query)
        rospy.loginfo(solution)

        test = crop(solution)
        rospy.loginfo(test)

        give_type = "has_type(" + str(test) + "," + "X)."
        rospy.loginfo(give_type)

        ref = prolog.once(give_type)
        rospy.loginfo(crop_plus(crop(ref)))
        return solution

#########################################################################
# crop magic

# crop unnecessary chars and begin after the ':'
def crop(String):
    dpunkt_index = str(String).find(":")
    if dpunkt_index != -1:
        ex_string = str(String)[dpunkt_index +1:]
        print("ex_string:" + ex_string)

        de_string = ex_string.strip(' "').rstrip('}')
        print("de_string:" + de_string)

        return de_string


# crop everything before the '#'
def crop_plus(string):
    # search '#'
    hi = str(string).find('#')

    if hi != -1:
        new_str = str(string)[hi + 1:]
        return new_str
    else:
        print("No '#'")
