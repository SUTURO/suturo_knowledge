#!/usr/bin/env python
import rospy
import rosprolog_client
from suturo_nlg.msg import KeyValuePair, MeaningRepresentation

prolog = rosprolog_client.Prolog()


def is_there(location, item):
    if item == "red pringles can":
        item = "Pringlesoriginal"
    elif item == "blue pringles can":
        item = "Pringlessaltvinegar"
    elif item == "banana":
        item = "Banana"
    query_string = "rdfs_individual_of(_X,hsr_objects:'" + item + "'), rdf_has(_X, hsr_objects:'supportedBy',_Link), rdf_urdf_name(_Link,Name)."
    solutions = prolog.all_solutions(query_string)

    if not solutions:
        return False

    location = location.replace(" ", "_")
    for solution in solutions:
        if location in solution['Name']:
            return True
    return False


def what_is_on(location):
    rospy.wait_for_service('/rosprolog/query')

    if location == "shelf":
        query_string = "once((all_objects_in_whole_shelf(_Objs), member(Obj,_Objs)))."
    elif location == "big table":
        query_string = "once((rdf_urdf_name(_Table, table_0_center),rdf_has(Obj, hsr_objects:'supportedBy', _Table)))."
    elif location == "small table":
        query_string = "once((rdf_urdf_name(_Table, table_1_center),rdf_has(Obj, hsr_objects:'supportedBy', _Table)))."
    elif location == "ground":
        query_string = "once((rdf_has(hsr_objects:'supportedBy', ground)))."
    else:
        return False

    solutions = prolog.all_solutions(query_string)

    solution = solutions[0]['Obj'].split('#')[1].split('_')[0]
    return solution


def callback(data):
    datadict = {}
    for keyvaluep in data.role_values:
        datadict[keyvaluep.key] = keyvaluep.value

    if datadict.keys().__contains__("predicate"):

        if datadict["predicate"] == "what is on":

            if not datadict.keys().__contains__("location"):
                rospy.logerr("Message didn't contain location")
                return
            datadict["item"] = what_is_on(datadict["location"])

        elif datadict["predicate"] == "is there":

            if not datadict.keys().__contains__("item"):
                rospy.logerr("Message didn't contain item")
                return
            if not datadict.keys().__contains__("location"):
                rospy.logerr("Message didn't contain location")
                return

            if is_there(datadict["location"], datadict["item"]):
                datadict["answer"] = "yes"
            else:
                datadict["answer"] = "no"
        #TODO predicate "supposed to go
        else:
            rospy.logerr("Message didn't contain any known predicates")
            return

        datadict["question"] = datadict["predicate"]
        del datadict["predicate"]
        pub(datadict)
    else:
        rospy.logerr("Message didn't contain any predicates")


def pub(datadict):
    dict_meaning_rep = MeaningRepresentation()
    for key in datadict:
        nkvp = KeyValuePair()
        nkvp.key = key
        nkvp.value = datadict[key]
        dict_meaning_rep.role_values.append(nkvp)
    publisher.publish(dict_meaning_rep)


if __name__ == '__main__':
    rospy.init_node('knowledge_interface')
    rospy.Subscriber("kinterface_in", MeaningRepresentation, callback)
    publisher = rospy.Publisher('kinterface_out', MeaningRepresentation, queue_size=10)
    rospy.spin()
