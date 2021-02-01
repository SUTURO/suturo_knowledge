:- module(rooms,[
    get_door_state/2,
    get_all_door_states/1,
    update_door_state/2
]).


:- rdf_meta
    min_door_joint_angle(?).


get_door_state(Door, State) :-
    get_urdf_id(URDF),
    triple(DoorLink, hsr_objects:'isSurfaceType', Door),
    urdf_link_parent_joint(URDF, DoorLink, DoorJoint),
    triple(DoorJoint, hsr_rooms:'hasJointState', JointState),
    State = JointState > min_door_joint_angle.

get_all_door_states(States) :-
    States = [],
    forall(has_type(DoorInstance, hsr_rooms:'Door'), 
        get_door_state(DoorInstance, State),
        append(States, [DoorInstance, State], States)).

update_door_state(Door, NewJointState) :-
    get_urdf_id(URDF),
    triple(DoorLink, hsr_objects:'isSurfaceType', Door),
    urdf_link_parent_joint(URDF, DoorLink, DoorJoint),
    forall(triple(DoorJoint, hsr_rooms:'hasJointState', JointState), 
        tripledb_forget(Door, hsr_rooms:'hasJointState', JointState)).
    tell(triple(DoorJoint, hsr_rooms:'hasJointState', NewJointState)).


min_door_joint_angle(angle) :-
    angle = 1,22. % corresponds to 70 degree


    