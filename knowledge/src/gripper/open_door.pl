:- module(open_door,
    [
        door_handle_to_open_door/2,
        perceiving_pose_of_door/2,
        passing_pose_of_door/2,
        manipulating_pose_of_door/2,
        get_angle_to_open_door/2,
        passing_pose_of_passage/2,
        update_door_state_dynamic/2
    ]).


:- use_module(library('model/environment/doors'), 
    [
        door_handles/2, 
        update_door_paths/1, 
        inside_door_handle/2, 
        outside_door_handle/2
    ]).
:- use_module(library('locations/actual_locations'), [robot_in_room/1]).


%% door_handle_to_open_door(?Door, ?DoorHandle) is nondet
%
% True if DoorHandle is a door handle of Door and 
% DoorHanle is in the same room as the robot
%
% @param Door A door IRI, DoorHandle A door handle IRI
%
door_handle_to_open_door(Door, DoorHandle) :-
    door_handles(Door, DoorHandles),
    member(DoorHandle, DoorHandles),
    robot_in_room(Room),
    has_urdf_name(DoorHandle, DoorHandleLink),
    get_urdf_origin(Map),
    tf_lookup_transform(Map, DoorHandleLink, pose(DoorHandlePosition, _)),
    position_in_room(DoorHandlePosition, Room).


%% perceiving_pose_of_door(?Door, ?Pose) is nondet
%
% Returns the Pose from which to perceive the state of door Door
%
% @param Door A door IRI, Pose Translation as [X, Y, Z] 
% and Rotation as [A, B, C, D]
%
perceiving_pose_of_door(Door, Pose) :-
    get_urdf_id(URDF),
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    urdf_link_collision_shape(URDF, DoorLink, box(Width, _, _), _),
    DeltaX is Width/2,
    Offset is Width + 0.2,
    tf_lookup_transform(DoorLink, 'base_footprint', pose([_, Y, _], _)),
    ((Y < 0)
    -> 
    (
        DeltaY is -1 * Offset,
        Angle is 0.0,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(DoorLink, Origin, pose([DeltaX, DeltaY, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    );
    (
        DeltaY is Offset,
        Angle is pi,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(DoorLink, Origin, pose([DeltaX, DeltaY, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    )).


%% passing_pose_of_door(?Door, ?Pose) is nondet
%
% Returns the Pose from which to pass the door Door
%
% @param Door A door IRI, Pose Translation as [X, Y, Z] 
% and Rotation as [A, B, C, D]
%
passing_pose_of_door(Door, Pose) :-
    get_urdf_id(URDF),
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    urdf_link_collision_shape(URDF, DoorLink, box(Width, _, _), _),
    tf_lookup_transform('base_footprint', DoorLink, pose([_, Y, _], _)),
    DeltaX is Width/2,
    ((Y < 0)
    -> 
    (
        DeltaY is 0.4,
        Angle is 0.0,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(DoorLink, Origin, pose([DeltaX, DeltaY, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    );
    (
        DeltaY is -0.4,
        Angle is pi,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(DoorLink, Origin, pose([DeltaX, DeltaY, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    )).



%% manipulating_pose_of_door(?Door, ?Pose) is nondet
%
% Returns the Pose from which to open the door Door
%
% @param Door A door IRI, Pose Translation as [X, Y, Z] 
% and Rotation as [A, B, C, D]
%
manipulating_pose_of_door(Door, Pose) :-
    get_urdf_origin(Origin),
    has_urdf_name(Door, DoorLink),
    tf_lookup_transform('base_footprint', DoorLink, pose([_, Y, _], _)),
    ((Y < 0)
    -> 
    (
        outside_door_handle(Door, OutsideDoorHandle),
        has_urdf_name(OutsideDoorHandle, OutsideDoorHandleLink),
        Angle is 0.0,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(OutsideDoorHandleLink, Origin, pose([0.0, -1.2, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    );
    (
        inside_door_handle(Door, InsideDoorHandle),
        has_urdf_name(InsideDoorHandle, InsideDoorHandleLink),
        Angle is pi,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(InsideDoorHandleLink, Origin, pose([0.0, 1.2, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    )).


%% passing_pose_of_passage(?Passage ?Pose) is nondet
%
% Returns the Pose from which to pass the passage Passage
%
% @param Passage A passage IRI, Pose Translation as [X, Y, Z] 
% and Rotation as [A, B, C, D]
%
passing_pose_of_passage(Passage, Pose) :-
    get_urdf_id(URDF),
    get_urdf_origin(Origin),
    has_urdf_name(Passage, PassageLink),
    tf_lookup_transform(PassageLink, 'base_footprint', pose([X, _, _], _)),
    ((X < 0)
    ->
    (
        Angle is 0.0,
        angle_to_quaternion(Angle, DeltaRotation),
        DeltaX is 0.4,
        tf_transform_pose(PassageLink, Origin, pose([DeltaX, 0.0, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    );
    (
        Angle is pi,
        angle_to_quaternion(Angle, DeltaRotation),
        DeltaX is -0.4,
        tf_transform_pose(PassageLink, Origin, pose([DeltaX, 0.0, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation]
    )).



%% get_angle_to_open_door(?Door, ?Angle) is nondet
%
% Returns the angle to fully open door Door
%
% @param Door A door IRI, Angle double
%
get_angle_to_open_door(Door, Angle) :-
    get_urdf_id(URDF),
    door_joint(Door, DoorJoint),
    has_urdf_name(DoorJoint, DoorJointName),
    urdf_joint_hard_limits(URDF, DoorJointName, [_, UpperLimit], _, _),
    triple(DoorJoint, soma:'hasJointState', DoorJointState),
    triple(DoorJointState, soma:'hasJointPosition', JointPosition),
    Angle is UpperLimit - JointPosition.



%% update_door_state_dynamic(?Door, ?Angle) is nondet
%
% Rotates door Door by angle Angle around its hinge
%
% @param Door A door IRI, Angle double
%
update_door_state_dynamic(Door, Angle) :-
    door_joint(Door, DoorJoint),
    triple(DoorJoint, soma:'hasJointState', DoorJointState),
    triple(DoorJointState, soma:'hasJointPosition', CurrentAngle),
    NewAngle is CurrentAngle + Angle,
    ( is_valid_joint_state(DoorJoint, NewAngle)
    -> 
    (
        forall(triple(DoorJointState, soma:'hasJointPosition', _), tripledb_forget(DoorJointState, soma:'hasJointPosition', _)),
        tell(triple(DoorJointState, soma:'hasJointPosition', NewAngle)),
        door_hinge(Door, DoorHinge),
        rotate_door_by_angle(Door, DoorHinge, Angle),
        update_door_paths(Door)
    )
    ; fail
    ).


%% update_door_state_measurement(?DoorHandle, ?Width, ?Depth) is nondet
%
% Calculates perceived opening angle from Width and Depth and Rotates
% Door by its angle around its hinge
%
% @param DoorHandle A door handle IRI, Width double, Depth double
%
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


%% rotate_door_by_angle(?Door, ?Hinge, ?Angle) is nondet
%
% Rotates door Door by angle Angle around hinge Hinge
%
% @param Door A door IRI, Hinge a hinge IRI, Angle double
%
rotate_door_by_angle(Door, Hinge, Angle) :-
    has_urdf_name(Door, DoorLink),
    has_urdf_name(Hinge, HingeLink),
    urdf_tf_frame(Door, DoorLinkWithPrefix),
    urdf_tf_frame(Hinge, HingeLinkWithPrefix),
    angle_to_quaternion(Angle, Rotation),
    tf_lookup_transform(DoorLink, HingeLink, pose(CurrentPos, _)),
    tf_transform_quaternion(HingeLink, DoorLink, Rotation, NewRotation),
    tell(is_at(DoorLink, [HingeLink, CurrentPos, NewRotation])),
    tell(is_at(DoorLinkWithPrefix, [HingeLinkWithPrefix, CurrentPos, NewRotation])).


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
    has_urdf_name(DoorJoint, DoorJointName),
    urdf_joint_hard_limits(URDF, DoorJointName, [LowerLimit, UpperLimit], _, _),
    ( JointState > LowerLimit, JointState < UpperLimit
    -> true
    ; fail
    ).
