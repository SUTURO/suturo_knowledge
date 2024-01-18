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

        print(parts)
        
        # remove unnecessary chars such as ":" and apostrophes
        name_part = parts[0].split(": ")[1].strip(' "').lower() if len(parts) > 0 else ""
        drink_part = parts[1]
        id_part = parts[2].strip(' "')

        print("Name:", name_part)
        print("Drink:", drink_part)
        print ("ID:", id_part)

        # check if a fav drink already exists
        query_check = "fav_drink(" + name_part + ", X)."
        check_for_drink = prolog.once(query_check)

        rospy.loginfo(check_for_drink)
        if check_for_drink == []:

            id = id_part

            if drink_part == "Coffee":
                query = "save_me_and_coffee(" + name_part + "," + "\'" + id + "\')."
                rospy.loginfo(query)
                prolog.once(query)

            elif drink_part == "RaspberryJuice":
                query = "save_me_and_raspberryjuice(" + name_part  + "," + "\'" + id + "\')."
                rospy.loginfo(query)
                prolog.once(query)

            elif drink_part == "Milk":
                query = "save_me_and_milk(" + name_part  + "," + "\'" + id + "\')."
                rospy.loginfo(query)
                prolog.once(query)

            elif drink_part == "Tea":
                query = "save_me_and_tea(" + name_part  + "," + "\'" + id + "\')."
                rospy.loginfo(query)
                prolog.once(query)

            else: 
                return ("sorry " + name_part.capitalize() + " but we don't know a drink named like "
                        + drink_part) 

            return ("Your name is " + name_part.capitalize() 
                    + " and your favourite drink is " + drink_part 
                    + "." + " We saved your information!"
            )
        else : 
            rospy.loginfo("We already registered your favourite drink")
            return ("We already registered your favourite drink as " + crop_plus(str(check_for_drink)) + ".")
        

#########################################################################
# 2:
# Is person X already known to us?
# used by: name_server
        
# later: if yes:return ID (X = name) or name (X = ID)

class InterfaceDoWeKnowYou:

    def do_we_known_u(self, name):
        rospy.loginfo("Second Interface is working.")

        # crop the input string to a useful string
        crop_string = crop(name).lower()
        count = 1

        query = "is_customer("+ crop_string +")."
        rospy.loginfo(query)
        solution = prolog.once(query)

        # save only when name is not already known
        if solution == dict():
            rospy.loginfo("Welcome back " + crop_string.capitalize() + "!")
            return True
        else:
            count += 1
            save = "save_me("+ crop_string + "," + str(count) + ")."
            rospy.loginfo(save)

            save_call = prolog.once(save)
            
            rospy.loginfo("We saved you!")
            rospy.loginfo("Nice to meet you " + crop_string.capitalize() + "!")

            # test if saving was successful
            test = "is_customer("+crop_string+")."
            rospy.loginfo("Test:" + str(test))
            test_call = prolog.once(test)
            if test_call != dict():
                rospy.loginfo("test_call: not successful")
            else: 
                rospy.loginfo("test_call: successful")

            # because person was not known previously
            return False

#########################################################################
# 2.5: 
# What's person X's ID? --> return ID
# What is the name X of person with ID Y? --> return name
# used by: name_server
    
class InterfaceGivePersonID:

    def whats_your_id(self, name):
        rospy.loginfo("whats your id - working")

        crop_name = crop(name).lower()

        query = "has_id(" + crop_name + ", X)."
        rospy.loginfo(query)

        sol = prolog.once(query)
        rospy.loginfo(sol)


        return sol
    

    def whats_your_name(self, guest_id):
        rospy.loginfo("whats your name - called")

        rospy.loginfo(guest_id)

        gid = crop(guest_id)

        name_q = "has_name(\'" + str(gid) + "\',X)."
        rospy.loginfo(name_q)

        name_ans = prolog.once(name_q)
        rospy.loginfo(name_ans)

        only_name = crop(name_ans)
        rospy.loginfo(name_ans)

        #the_name = str(only_name).replace("\'", "")
        #rospy.loginfo(the_name)

        return name_ans
    
#########################################################################
# 3: 
# What's person X's favourite drink?
# used by: drink_server

class InterfacePersonAndFavDrink:

    def what_is_your_fav_drink(self, name):
        rospy.loginfo("Third Interface is working.")

        # crop the input string to a useful string
        crop_string = crop(str(name)).lower()
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
        sol = crop_plus(crop(ref))
        rospy.loginfo(sol)

        return sol.replace("\'", "")

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
