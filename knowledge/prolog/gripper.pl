:- module(gripper,
    [
    all_objects_in_gripper/1,
    gripper/1,
    gripper_init/1,
    attach_object_to_gripper/1,
    release_object_from_gripper/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).


:- rdf_meta
    gripper(+),
    gripper_int(r),
    attach_object_to_gripper(r),
    release_object_from_gripper(r)
    .




gripper(Gripper) :-
    Gripper = gripper.

gripper_init(Gripper) :-
    %rdf_instance_from_class(knowrob:'EnduringThing-Localized', belief_state, Gripper),
    rdf_assert(Gripper, rdf:type, owl:'NamedIndividual', belief_state),
    rdf_assert(Gripper, knowrob:'frameName', hand_palm_link, belief_state).


all_objects_in_gripper(Instances):-
    findall(Instance, (
        objects_on_surface(Objs, gripper),
        member(Instance, Objs)
        ), Instances).

attach_object_to_gripper(Instance) :-
    rdf_retractall(Instance, hsr_objects:'supportedBy', _),
    gripper(Gripper),
    rdf_assert(Instance, hsr_objects:'supportedBy', Gripper),
    object_frame_name(Instance, InstanceFrame),
    object_frame_name(Gripper,GripperFrame),
    hsr_lookup_transform(GripperFrame, InstanceFrame, PoseTrans, PoseRota),
    belief_at_update(Instance, [GripperFrame, _, PoseTrans, PoseRota]).

release_object_from_gripper([NewPose,NewRotation]) :-
    gripper(Gripper),
    objects_on_surface(Instances, Gripper),
    member(Instance, Instances),
    %object_frame_name(Instance, InstanceFrame),
    hsr_belief_at_update(Instance, [map, _, NewPose, NewRotation]),
    rdf_retractall(Instance, hsr_objects:'supportedBy', _),
    position_supportable_by_surface(NewPose, NewSurface),
    rdf_assert(Instance, hsr_objects:'supportedBy', NewSurface),
    group_target_objects.
