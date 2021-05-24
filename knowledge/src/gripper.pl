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
    findall(Instance, (
        has_location(Instance, ObjectLocation),
        triple(ObjectLocation, hsr_objects:'supportedBy', Gripper)
        ), Instances).

attach_object_to_gripper(Instance) :-
    %forall(triple(Instance, hsr_objects:'supportedBy', _), tripledb_forget(Instance, hsr_objects:'supportedBy', _)),
    forget_object_at_location(Instance),
    gripper(Gripper),
    has_location(Instance, InstanceLocation),
    tell(triple(InstanceLocation, hsr_objects:'supportedBy', Gripper)),
    %object_frame_name(Instance, InstanceFrame),
    %object_frame_name(Gripper,GripperFrame),
    object_tf_frame(Instance,InstanceFrame),
    hsr_lookup_transform(Gripper, InstanceFrame, PoseTrans, PoseRota),
    tell(is_at(Instance, [Gripper, PoseTrans, PoseRota])),
    republish, republish.

release_object_from_gripper([NewPose,NewRotation]) :-
    gripper(Gripper),
    objects_on_surface(Instances, Gripper),
    member(Instance, Instances),
    %object_frame_name(Instance, InstanceFrame),
    %hsr_belief_at_update(Instance, [map, _, NewPose, NewRotation]),
    tell(is_at(Instance, ['map', NewPose, NewRotation])),
    forget_object_at_location(Instance),
    at_location(Instance, _, _, _),
    group_target_objects,
    republish, republish.
