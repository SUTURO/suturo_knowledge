:- module(gripper_info,[
    init_gripper/0,
    all_objects_in_gripper/1
]).
:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).


:- use_module(library('ros/marker/marker_plugin'), [republish/0]).
:- use_module(library('model/objects/object_info'), [is_suturo_object/1]).

%% init_gripper is det.
%
% Initialize gripper in the environment.
%
%
init_gripper :-
    gripper(Gripper),
    tell(has_type(Gripper, owl:'NamedIndividual')),
    tell(triple(Gripper, knowrob:'frameName', hand_palm_link)).

%% all_objects_in_gripper(Instances) is nondet.
%
% Returns all objects held in gripper.
%
% @param Instances the variable list to be filled
%
all_objects_in_gripper(Instances):-
    gripper(Gripper),
    findall(Object, (
        is_suturo_object(Object),
        once(has_location(Object, ObjectLocation)),
        triple(ObjectLocation, soma:'isSupportedBy', Gripper)
        ), Instances).


gripper(Gripper) :-
    Gripper = hand_palm_link.