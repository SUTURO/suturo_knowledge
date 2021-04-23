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
        create_door(Link, Door),
        create_door_joint(Door, Link),
        create_door_hinge(Door, Link),
        create_door_handles(Door, Link),
        assign_connecting_rooms(Door, Link)
    )).


create_door(DoorLink, Door) :-
    tell(has_type(Door, hsr_rooms:'Door')),
    tell(triple(Door, urdf:'hasURDFName', DoorLink)),


create_door_joint(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, DoorJointName),
    tell(has_type(DoorJoint, soma:'Joint')),
    tell(triple(DoorJoint, urdf:'hasURDFName', DoorJointName)),
    tell(has_type(DoorJointState, soma:'JointState')),
    tell(triple(DoorJoint, soma:'hasJointPosition', DoorJointState)),
    tell(triple(DoorJointState, soma:'hasJointPosition', 0.0)).


create_door_hinge(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, Door, HingeJointName),
    urdf_joint_parent_link(URDF, HingeJoint, HingeLink).
    tell(has_type(Hinge, hsr_rooms:'Hinge')),
    tell(triple(Hinge, urdf:'hasURDFName', HingeLink)),
    tell(has_type(HingeJoint, urdf:'HingeJoint')),
    tell(triple(HingeJoint, urdf:'hasURDFName', HingeJointName)),
    tell(triple(Door, knowrob:'doorHingedTo', Hinge)).


create_door_handles(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_child_joints(URDF, DoorLink, ChildJoints),
    forall(member(ChildJoint, ChildJoints),
    (
        urdf_joint_child_link(URDF, ChildJoint, DoorHandleLink),
        tell(has_type(DoorHandle, hsr_rooms:'DoorHandle')),
        tell(has_urdf_name(DoorHandle, DoorHandleLink)),
        (sub_string(DoorHandleLink,_,_,_,"inside")
        ->
        (
            tell(has_type(InsideHandleLocation, soma:'Location')),
            tell(has_location(DoorHandle, InsideHandleLocation)),
            tell(triple(InsideHandleLocation, soma:'isInsideOf', Door))
        );
        (
            tell(has_type(OutsideHandleLocation, soma:'Location')),
            tell(has_location(DoorHandle, OutsideHandleLocation)),
            tell(triple(OutsideHandleLocation, hsr_rooms:'isOutsideOf', Door))
        ))
    )).
    

create_passage(Passage, PassageLink) :-
    tell(has_type(Passage, hsr_rooms:'Passage')),
    tell(triple(Passage, urdf:'hasURDFName', PassageLink)).


assign_connecting_rooms(RoomLinkage, RoomLinkageLink) :-
    split_string(RoomLinkageLink, "_", "", [_,ExpRoom1Link, ExpRoom2Link, _]),
    has_type(Room1, hsr_rooms:'Room'),
    has_type(Room2, hsr_rooms:'Room'),
    urdf_room_center_link(Room1, ActRoom1Link),
    urdf_room_center_link(Room2, ActRoom2Link),
    sub_string(ActRoom1Link,_,_,_,ExpRoom1Link),
    sub_string(ActRoom2Link,_,_,_,ExpRoom2Link),
    tell(has_type(Location, soma:'Location')),
    tell(triple(Location, soma:'isLinkedTo', Room1)),
    tell(triple(Location, soma:'isLinkedTo', Room2)),
    tell(triple(RoomLinkage, dul:'has_location', Location)).


perceiving_pose_of_door(Door, Pose) :-
    get_urdf_id(URDF),
    get_urdf_origin(Origin),
    triple(Door, urdf:'hasURDFName', DoorLink),
    urdf_link_collision_shape(URDF, DoorLink, box(Width, _, _), _),
    tf_transform_point(DoorLink, Origin, [0, Width/2, 0], [NewX, NewY, _]),
    Offset is Width + 0.2,
    Position = [NewX+Offset, NewY, 0.0],
    tf_lookup_transform('base_footprint', DoorLink, pose([X, _, _], _)),
    tf_lookup_transform(Origin, DoorLink, pose(_, Rotation)),
    ((X < 0)
    -> 
    (
        Pose = [Position, Rotation]
    );
    (
        rotation_opposite_to_door(Door, NewRotation),
        Pose = [Position, NewRotation]
    ).


manipulating_pose_of_door(Door, Pose) :-
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    tf_lookup_transform('base_footprint', DoorLink, pose([X, _, _], _)),
    tf_lookup_transform(Origin, DoorLink, pose(_, Rotation)),
    ((X < 0)
    -> 
    (
        outside_door_handle(Door, OutsideDoorHandle),
        has_urdf_name(OutsideDoorHandle, OutsideDoorHandleLink)
        tf_lookup_transform(Origin, OutsideDoorHandleLink, pose([X, Y, _], _)),
        Pose = [[X-1.2, Y, 0.0], Rotation]
    );
    (
        inside_door_handle(Doo, InsideDoorHandle),
        has_urdf_name(InsideDoorHandle, InsideDoorHandleLink),
        tf_lookup_transform(Origin, InsideDoorHandleLink, pose([X, Y, _], _)),
        rotation_opposite_to_door(Door, OppositeRotation),
        Pose = [[X+1.2, Y, 0.0], OppositeRotation]
    )).


rotation_aligned_with_door(Door, Rotation) :-
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    tf_lookup_transform(Origin, DoorLink, pose(_, Rotation)).


rotation_opposite_to_door(Door, Rotation) :-    
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    tf_lookup_transform(Origin, DoorLink, pose(_, DoorRotation)),
    Angle is 0.5 * pi
    angle_to_quaternion(Angle, DeltaRotation),
    tf_transform_quaternion(DoorLink, Origin, DeltaRotation, Rotation).


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


angle_to_quaternion(Angle, Quaternion) :-
    C is cos(Angle/2),
    S is sin(Angle/2),
    Quaternion = [0.0, 0.0, S, C].



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


door_hinge(Door, Hinge) :-
    triple(Door, urdf:'doorHingedTo', Hinge).

inside_door_handle(Door, DoorHandle) :-
    triple(Location, soma:'isInsideOf', Door),
    has_location(DoorHandle, Location).

outside_door_handle(Door, DoorHandle) :-
    triple(Location, hsr_rooms:'isOutsideOf', Door),
    has_location(DoorHandle, Location).


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