:- module(doors,
    [
        init_doors/0,
        update_door_state/2,
        get_door_state/2,
        get_all_door_states/1
    ]).

:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).


%% init_doors
%  
%  Creates an object instance of concept door
%  for each door specified in the URDF
%  in the knowledge base and initializes
%  its opening angle to 0.0 (fully closed)
init_doors :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    forall(
    (
        member(Link, Links),
        sub_string(Link,_,_,_,door), 
        not sub_string(Link,_,_,_,handle), 
        not sub_string(Link,_,_,_,hinge)
    ),
    (
        tell(has_type(Link, hsr_rooms:'Door')),
        urdf_link_parent_joint(URDF, Link, DoorJoint),
        tell(triple(DoorJoint, hsr_rooms:'hasJointState', 0.0))
    )).


%% update_door_state(Door, Angle)
%
%  Sets the opening angle of the specified
%  door to the specified angle.
%
%  @param Door Name of the door of which the 
%  opening angle should be updated as a string
%  @param Angle New value of the opening angle as float
update_door_state(Door, Angle) :-
    object_frame_name(Door, DoorName),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorName, DoorJoint),
    ( is_valid_joint_state(DoorJoint, Angle)
    -> 
    (
        forall(triple(DoorJoint, hsr_rooms:'hasJointState', _), tripledb_forget(DoorJoint, hsr_rooms:'hasJointState', _)),
        tell(triple(DoorJoint, hsr_rooms:'hasJointState', Angle))
    )
    ; fail
    ).


%% get_door_state(Door, DoorState)
%  
%  returns the state of the specified door
%
%  @param Door Name of the door as a string
%  @param DoorState Sate of the door i.e.:
%  0 - closed
%  1 - open
get_door_state(Door, DoorState) :-
    object_frame_name(Door, DoorName),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorName, DoorJoint),
    triple(DoorJoint, hsr_rooms:'hasJointState', JointState),
    min_door_joint_angle(MaxAngle),
    MinAngle is -1*MaxAngle,
    ( JointState < MaxAngle, JointState > MinAngle 
    -> DoorState = 0
    ; DoorState = 1
    ).


%% get_all_door_states(DoorStates)
%
%  returns a list of all doors and their states 
%
%  @param DoorStates List of doors and their
%  corresponding state i.e.:
%  0 - closed
%  1 - open
get_all_door_states(DoorStates) :-
    findall([Door, State], 
    (
        has_type(Door, hsr_rooms:'Door'),
        get_door_state(Door, State)
    ), 
        DoorStates
    ).



%% is_valid_joint_state(DoorJoint, JointState)
%  
%  checks whether a joint movement is valid/ possible
%  for the specified joint
%
%  @param DoorJoint Joint for which a movement should
%  be checked
%  @param JointState movement the specified joint should 
%  perform e.g.: an angle for a revolute joint
is_valid_joint_state(DoorJoint, JointState) :-
    get_urdf_id(URDF),
    urdf_joint_hard_limits(URDF, DoorJoint, [LowerLimit, UpperLimit], _, _),
    ( JointState > LowerLimit, JointState < UpperLimit
    -> true
    ; fail
    ).


%% min_door_joint_angle(Angle)
%
%  returns the the minimum opening angle
%  of a door to be seen as open
%
%  @param Angle Minimum opening angle
min_door_joint_angle(Angle) :-
    Angle = 1.22. % corresponds to 70 degree