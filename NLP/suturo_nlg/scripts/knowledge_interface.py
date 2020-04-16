#!/usr/bin/env python
import rospy
import rosprolog_client
from suturo_nlg.msg import KeyValuePair, MeaningRepresentation

prolog = rosprolog_client.Prolog()


def item_translator(item):
    if item == "red pringles can":
        return "Pringlesoriginal"
    elif item == "blue pringles can":
        return "Pringlessaltvinegar"
    elif item == "banana":
        return "Banana"
    elif item == "cup":
        return "Cup"
    else:
        return item.capitalize()


def errordict():
    dict = {"error": "Something went wrong"}
    return dict


def is_there(location, item):
    item_t = item_translator(item)
    query_string = "rdfs_individual_of(_X,hsr_objects:'" + item_t + "'), rdf_has(_X, hsr_objects:'supportedBy',_Link), rdf_urdf_name(_Link,Name)."
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


def supposed_to_go(item):
    item_t = item_translator(item)
    query_string = "hsr_existing_objects(Objs), member(Obj,Objs), rdfs_individual_of(Obj, hsr_objects:'" + item_t + "'), object_goal_surface(Obj,_Surf), rdf_urdf_name(_Surf,SurfName)"
    solutions = prolog.all_solutions(query_string)
    if not solutions:
        return False
    solution = solutions[0]['SurfName'].split('_')[0]
    return solution


def callback(data):
    datadict = {}
    for keyvaluep in data.role_values:
        datadict[keyvaluep.key] = keyvaluep.value

    if datadict.keys().__contains__("predicate"):

        if datadict["predicate"] == "what is on":

            if not datadict.keys().__contains__("location"):
                rospy.logerr("Message didn't contain location")
                datadict = errordict()
                pub(datadict)
                return
            datadict["item"] = what_is_on(datadict["location"])

        elif datadict["predicate"] == "is there":

            if not datadict.keys().__contains__("item"):
                rospy.logerr("Message didn't contain item")
                datadict = errordict()
                pub(datadict)
                return
            if not datadict.keys().__contains__("location"):
                rospy.logerr("Message didn't contain location")
                datadict = errordict()
                pub(datadict)
                return
            if is_there(datadict["location"], datadict["item"]):
                datadict["answer"] = "yes"
            else:
                datadict["answer"] = "no"

        elif datadict["predicate"] == "supposed to go":
            if not datadict.keys().__contains__("item"):
                rospy.logerr("Message didn't contain item")
                datadict = errordict()
                pub(datadict)
                return
            if not supposed_to_go(datadict["item"]):
                datadict = errordict()
                pub(datadict)
                return
            datadict["location"] = supposed_to_go(datadict["item"])

        else:
            rospy.logerr("Message didn't contain any known predicates")
            datadict = errordict()
            pub(datadict)
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
