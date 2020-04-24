#!/usr/bin/env python
import rospy
import rospkg
import rosprolog_client
from suturo_nlg.msg import KeyValuePair, MeaningRepresentation

prolog = rosprolog_client.Prolog()
rospack = rospkg.RosPack()
wd = rospack.get_path('suturo_nlg')
filepath = wd + "/scripts/object_translate"


def item_translator_to_knowledge(item):
    with open(filepath) as f:
        for line in f:
            if item == line.split(",")[0]:
                return line.split(",")[1]
        pub(errordict("I have no idea what you mean with an object called " + item + " is"))
        return False


def item_translator_from_knowledge(item):
    with open(filepath) as f:
        for line in f:
            if item == line.split(",")[1]:
                return line.split(",")[0]
        return False


def errordict(text="Something went wrong"):
    return {"error": text}


def is_there(dict):
    if not dict.keys().__contains__("item"):
        rospy.logerr("Message didn't contain item")
        datadict = errordict("I didn't get the object you meant.")
        pub(datadict)
        return
    if not dict.keys().__contains__("location"):
        rospy.logerr("Message didn't contain location")
        datadict = errordict()
        pub(datadict)
        return
    item_t = item_translator_to_knowledge(dict["item"])
    query_string = "rdfs_individual_of(_X,hsr_objects:'" + item_t + "'), rdf_has(_X, hsr_objects:'supportedBy',_Link), rdf_urdf_name(_Link,Name)."
    solutions = prolog.all_solutions(query_string)

    if not solutions:
        dict["answer"] = "no"

    location = dict["location"].replace(" ", "_")
    # TODO this isn't nice
    for solution in solutions:
        if location in solution['Name']:
            dict["answer"] = "yes"
    dict["answer"] = "no"
    pub(dict)


def what_is_on(dict):
    if not dict.keys().__contains__("location"):
        rospy.logerr("Message didn't contain location")
        pub(errordict("What location did you mean?"))
        return False

    if dict["location"] == "hcr shelf":
        query_string = "once((hsr_existing_objects(_Objs), member(Obj,_Objs), rdf_has(Obj, hsr_objects:'supportedBy', _S), rdf_urdf_name(_S,_SN), sub_string(_SN,_,_,_,\"hcr_shefl\")))."
    elif dict["location"] == "bookshelf":
        query_string = "once((hsr_existing_objects(_Objs), member(Obj,_Objs), rdf_has(Obj, hsr_objects:'supportedBy', _S), rdf_urdf_name(_S,_SN), sub_string(_SN,_,_,_,\"bookshelf_floor\")))."
    elif dict["location"] == "big table":
        query_string = "once((rdf_urdf_name(_Table, table_center),rdf_has(Obj, hsr_objects:'supportedBy', _Table)))."
    elif dict["location"] == "small table":
        query_string = "once((rdf_urdf_name(_Table, bed_table_center),rdf_has(Obj, hsr_objects:'supportedBy', _Table)))."
    elif dict["location"] == "ground":
        query_string = "once((rdf_has(hsr_objects:'supportedBy', ground)))."
    else:
        rospy.logerr("Message didn't contain location")
        dict = errordict("What location did you mean?")
        pub(dict)
        return False
    solutions = prolog.all_solutions(query_string)
    if not solutions:
        pub(errordict("I don't know of any objects on the " + dict["location"]))
        return
    dict["item"] = item_translator_from_knowledge(solutions[0]['Obj'].split('#')[1].split('_')[0])
    pub(dict)
    return True


def supposed_to_go(dict):
    if not dict.keys().__contains__("item"):
        rospy.logerr("Message didn't contain item")
        datadict = errordict("What item did you mean?")
        pub(datadict)
        return
    item_t = item_translator_to_knowledge(dict["item"])
    query_string = "hsr_existing_objects(Objs), member(Obj,Objs), rdfs_individual_of(Obj, hsr_objects:'" + item_t + "'), object_goal_surface(Obj,_Surf), rdf_urdf_name(_Surf,SurfName)"
    solutions = prolog.all_solutions(query_string)
    if not solutions:
        pub(errordict("I don't know where that item is supposed to go"))
    dict["location"] = solutions[0]['SurfName'].split('_')[0]
    pub(dict)


def callback(data):
    datadict = {}
    for keyvaluep in data.role_values:
        datadict[keyvaluep.key] = keyvaluep.value

    if datadict.keys().__contains__("predicate"):
        if datadict["predicate"] == "what is on":
            what_is_on(datadict)
        elif datadict["predicate"] == "is there":
            is_there(datadict)
        elif datadict["predicate"] == "supposed to go":
            supposed_to_go(datadict)
        else:
            rospy.logerr("Message didn't contain any known predicates")
            datadict = errordict("The last sentence was not a question that I can understand")
            pub(datadict)
            return
    else:
        rospy.logerr("Message didn't contain any predicates")


def pub(datadict):
    if datadict.keys().__contains__("predicate"):
        datadict["question"] = datadict["predicate"]
        del datadict["predicate"]

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
    rospy.loginfo("Knowledge Interface: Waiting for rosprolog service")
    rospy.wait_for_service('/rosprolog/query')
    rospy.loginfo("Knowledge Interface: rosprolog service is running knowledge interface now running")
    rospy.spin()
