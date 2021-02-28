:- module(doors,
    [
        init_doors/0,
        update_door_state/2,
        update_door_state_dynamic/2,
        update_door_state_measurement/3,
        get_door_state/2,
        get_all_door_states/1,
        get_angle_to_open_door/2
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
    triple(DoorJoint, hsr_rooms:'hasJointState', CurrentAngle),
    NewAngle is CurrentAngle + Angle,
    ( is_valid_joint_state(DoorJoint, NewAngle)
    -> 
    (
        forall(triple(DoorJoint, hsr_rooms:'hasJointState', _), tripledb_forget(DoorJoint, hsr_rooms:'hasJointState', _)),
        tell(triple(DoorJoint, hsr_rooms:'hasJointState', NewAngle))
    )
    ; fail
    ).
    

update_door_state_dynamic(Door, Angle) :-
    object_frame_name(Door, DoorName),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorName, DoorJoint),
    triple(DoorJoint, hsr_rooms:'hasJointState', CurrentAngle),
    NewAngle is CurrentAngle + Angle,
    ( is_valid_joint_state(DoorJoint, NewAngle)
    -> 
    (
        forall(triple(DoorJoint, hsr_rooms:'hasJointState', _), tripledb_forget(DoorJoint, hsr_rooms:'hasJointState', _)),
        tell(triple(DoorJoint, hsr_rooms:'hasJointState', NewAngle)),
        get_door_hinge(Door, DoorHinge),
        rotate_door_by_angle(Door, DoorHinge, Angle)
    )
    ; fail
    ).
    


rotate_door_by_angle(Door, DoorHinge, Angle) :-
    get_urdf_origin(Origin),
    C is cos(Angle/2), S is sin(Angle/2),
    Rotation = [0.0, 0.0, S, C],
    tf_lookup_transform(Door, DoorHinge, pose(CurrentPos, _)),
    tf_transform_quaternion(DoorHinge, Door, Rotation, NewRotation),
    tell(is_at(Door, [DoorHinge, CurrentPos, NewRotation])).



get_angle_to_open_door(Door, Angle) :-
    object_frame_name(Door, DoorName),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorName, DoorJoint),
    urdf_joint_hard_limits(URDF, DoorJoint, [LowerLimit, UpperLimit], _, _),
    triple(DoorJoint, hsr_rooms:'hasJointState', CurrentAngle),
    get_door_hinge(Door, DoorHinge),
    tf_lookup_transform('base_footprint', DoorHinge, pose([_, Y, _], _)),
    ( Y < 0
    -> Angle is LowerLimit - CurrentAngle
    ; Angle is UpperLimit - CurrentAngle
    ).



update_door_state_measurement(DoorHandle, Width, Depth) :-
    get_urdf_id(URDF),
    get_urdf_origin(Origin),
    urdf_link_parent_joint(URDF, DoorHandle, DoorHandleJoint),
    urdf_joint_parent_link(URDF, DoorHandleJoint, Door),
    urdf_link_parent_joint(URDF, Door, DoorHingeJoint),
    urdf_joint_parent_link(URDF, DoorHingeJoint, DoorHinge),
    tf_lookup_transform(DoorHinge, Origin, pose([XHinge, YHinge, _], _)),
    tf_lookup_transform(DoorHandle, Origin, pose([XHandle, YHandle, _], _)),
    HDX is Width - XHinge,
    HDY is Depth - YHinge,
    sqrt(((HDX*HDX) + (HDY*HDY)), Hyp),
    KDX is (Width - XHandle)/2,
    KDY is (Depth - YHandle)/2,
    sqrt(((KDX*KDX) + (KDY*KDY)), Kath),
    Angle is 2*sin(Kath/Hyp),
    update_door_state_dynamic(Door, Angle).

    

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



get_door_hinge(Door, Hinge) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, Door, HingeJoint),
    urdf_joint_parent_link(URDF, HingeJoint, Hinge).


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