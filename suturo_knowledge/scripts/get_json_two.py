#!/usr/bin/env python3
import rospy
import json
from std_msgs.msg import String
# from nlp_msgs_to_parse.msg import JsonFile
import rosprolog_client

#prolog = rosprolog_client.Prolog()


def callback(String):
    # load converted data into python dictonary
    with open('nlp.json') as nlp:
        # convert json string to phyton object
        json_data = json.load(nlp)

    # extract the names and drinks from the json file
    # in list of lists, print first entry of first list
    # example for nlp.json: '{"name": {robert, lisa, jonas}, "fav_drink": {cola, cider, limo}}'
    print(json_data['entities'][0]['NaturalPerson'])
    print(json_data['entities'][0]['drink'])

    # NaturalPerson: name
    # drink: fav_drink

    # Tupel = {'text':'put the fork in the dishwasher.', 'intent': 'Cleaning', 'entities': {('NaturalPerson', 'Bob')}}

    len_names = len(json_data['names'])
    len_drinks = len(json_data['fav_drinks'])

    # checking if number of names and number of fav_drink is the same
    if len_names == len_drinks:
        # iterate over every name in names and create person-object
        # TODO: add all parameters for create_object
        i = 0
        for name in json_data['names']:
            query1 = "save_object(" + json_data['names'][name] + ")."
            # query that adds a predicate to an object
            # format: fav_drink(robert, cola).
            # TODO: check, ob das so mit dem i funktioniert
            query2 = "add_predicate_to_object(" + json_data['names'][name] + json_data['fav_drink'][i] + "))"
            i += 1

    else:
        rospy.loginfo("I don't have enough information :/")

        json_data['entities'][0][0]

    # überlegen: wenn '{"name": robert,lisa,jonas], "fav_drink": {cola, NULL, limo}}', wie wir damit umgehen

    rospy.loginfo(query1)
    rospy.loginfo(query2)

    p1_once = prolog.once(query1)
    p2_once = prolog.once(query2)
    rospy.loginfo(p1_once)
    rospy.loginfo(p2_once)

    # pub.publish(msg_object)


if __name__ == '__main__':
    try:
        rospy.init_node('suturo_get_info_parser')
        # start publisher node
        pub = rospy.Publisher('nlp_parser_output', String, queue_size=10)

        # start subscriber node
        rospy.Subscriber('from_nlp', String, callback)
        rospy.loginfo('string read')
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
