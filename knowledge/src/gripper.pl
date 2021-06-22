:- module(gripper,
    [
    all_objects_in_gripper/1,
    gripper/1,
    gripper_init/1,
    attach_object_to_gripper/1,
    release_object_from_gripper/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).
:- use_module(library('ros/marker/marker_plugin'), [republish/0]).

:- rdf_meta
    gripper(+),
    gripper_int(r),
    attach_object_to_gripper(r),
    release_object_from_gripper(r)
    .




gripper(Gripper) :-
    %Gripper = gripper.
    Gripper = hand_palm_link.

gripper_init(Gripper) :-
    %rdf_instance_from_class(knowrob:'EnduringThing-Localized', belief_state, Gripper),
    tell(has_type(Gripper, owl:'NamedIndividual')),
    tell(triple(Gripper, knowrob:'frameName', hand_palm_link)).


all_objects_in_gripper(Instances):-
    gripper(Gripper),
    findall(Object, (
        is_suturo_object(Object),
        once(has_location(Object, ObjectLocation)),
        triple(ObjectLocation, soma:'isSupportedBy', Gripper)
        ), Instances).

attach_object_to_gripper(Object) :-
    forget_object_at_location(Object),
    gripper(Gripper),
    has_location(Object, ObjectLocation),
    tell(triple(ObjectLocation, soma:'isSupportedBy', Gripper)),
    object_tf_frame(Object,ObjectFrame),
    hsr_lookup_transform(Gripper, ObjectFrame, PoseTrans, PoseRota),
    tell(is_at(Object, [Gripper, PoseTrans, PoseRota])),
    republish, republish.

release_object_from_gripper([NewPose,NewRotation]) :-
    all_objects_in_gripper(Instances),
    member(Instance, Instances),
    tell(is_at(Instance, ['map', NewPose, NewRotation])),
    forget_object_at_location(Instance),
    object_at_location(Instance, _, _, _),
    set_object_handeled(Instance),
    republish, republish.
