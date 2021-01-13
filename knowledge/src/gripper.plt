:- begin_tests('gripper').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('knowrob')).
:- use_module(library('lang/terms/holds')).
:- use_module(library('model/RDFS')).
:- use_module(library('lang/terms/is_a')).

:- use_module(library('surfaces')).

:- include(gripper).



add_object1:-
	tell(instance_of(Banana, hsr_objects:'Banana')).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%% ACTUAL TESTS %%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(initGripper) :-
    gripper_init(Gripper),
    has_type(Gripper,owl:'NamedIndividual'),
    holds(Gripper,knowrob:'frameName',hand_palm_link).


test(objectsInGripper) :-
    all_objects_in_gripper([]).


test(attachObject, [setup(add_object1)]) :-
    instance_of(A,hsr_objects:'Banana'),
    attach_object_to_gripper(A).

:- end_tests(gripper).