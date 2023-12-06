#!/usr/bin/env python3

import rospy
import rosprolog_client
import json
from std_msgs.msg import String
#from nlp_msgs.msg import Dictonary

prolog = rosprolog_client.Prolog()


def process_input(json_data):

    rospy.loginfo("Message: " + json_data.data + " received!")

    try:
        # Lade die JSON-Daten
        json_data_dict = json.loads(json_data.data)

    except json.JSONDecodeError:
        rospy.logerr("Invalid JSON format in message: {}".format(json_data.data))
        return       
    


    # convert string to dictionary
    #the_dict = list(input_string.data)

    rospy.loginfo("String Converted to a dictonary!")

    # we want the entities
    if 'entities' in json:
        entities = json_data_dict['entities']
    
    output = "" + entities + ""
        
        # entities are written in tuples e.g. ('NaturalPerson', 'Lisa')
        # go through ever tuple of the class 'NaturalPerson' and add name to list 
     #   for entity in entities:
            # tuple assignment 
     #       entity_class, entity_object = entity

     #       all = [entities[1] for tup in entities] 
            
        #{ 'text': 'can you get me some milk from the fridge?', 
	    #  'intent': 'Fetching', 
	    #  'entities': 
	    #      {
		#         ('PhysicalArtifact', 'fridge'), 
		#         ('drink', 'milk')
	    #      }
        #}   

#[[1][2][3]]
        
    # we only want the second arguments (= objects) in the list of tuples under 'entity'

        #for obj in all_objects:
        #    query = "is_object("+ obj + ");"
        #    rospy.loginfo(query)
        #    sol = prolog.once(query)
        #    # wenn obj ein Object ist...
        #    if sol:
        #        rospy.loginfo("" + obj + "is an existing object.")

        #        # .. dann frage, ob es einen predefined_name gibt und dann...
        #        query2 = "object_has_predefined_name(" + obj + "," + "X);"
        #        rospy.loginfo(query2)
        #        # vllt sowas wie if name not null einbauen 
        #        su = prolog.once(query2)
        #        rospy.loginfo("The object" + obj + "has name" + su)

                # frage, welcher class das object zugeordnet ist 
        #        query3 = "has_class(" + obj + ");"
        #        rospy.loginfo(query3)

                #gib zurück: für jedes object die class in der Schreibweise:
                # {entities: ('class', obj)}

        #    else:
        #        rospy.loginfo("There is no such object as" + obj + "")
                # query3 = object anlegen mit gegebenen Namen 
        
            
    pub.publish(output)
                

#def service_callback(request):
#    nlp_input = request.input_data
#    process_input(nlp_input)
#
#    return ServiceResponse()
        

if __name__ == '__main__':
    rospy.init_node('get_object_info')
#    serv = rospy.Service('get_object_info', NewService, service_callback)
    rospy.Subscriber('whatever', String, process_input)
    pub = rospy.Publisher('/info', String, queue_size=10)
    rospy.loginfo('hello')
    rospy.spin()

        
