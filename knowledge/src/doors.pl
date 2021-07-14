:- module(doors,
    [
        init_doors/0,
        init_door_paths/0,
        is_door/1,
        is_passage/1,
        update_door_state/2,
        update_door_state_dynamic/2,
        update_door_state_measurement/3,
        get_door_state/2,
        get_all_door_states/1,
        get_angle_to_open_door/2,
        perceiving_pose_of_door/3,
        manipulating_pose_of_door/2,
        passing_pose_of_door/2,
        passing_pose_of_passage/2,
        shortest_path_between_rooms/3,
        inside_door_handle/2,
        outside_door_handle/2
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
        sub_string(Link,_,_,_,door_center)
    ),
    (
        create_door(Link, Door),
        create_door_joint(Link),
        create_door_hinge(Door, Link),
        create_door_handles(Door, Link),
        assign_connecting_rooms(Door, Link)
    )),
    forall((
        member(Link, Links),
        sub_string(Link,_,_,_,path)
    ),
    (
        create_passage(Passage, Link),
        assign_connecting_rooms(Passage, Link)
    )).


create_door(DoorLink, Door) :-
    tell(has_type(Door, hsr_rooms:'Door')),
    tell(triple(Door, urdf:'hasURDFName', DoorLink)).


create_door_joint(DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, DoorJointName),
    tell(has_type(DoorJoint, soma:'Joint')),
    tell(triple(DoorJoint, urdf:'hasURDFName', DoorJointName)),
    tell(has_type(DoorJointState, soma:'JointState')),
    tell(triple(DoorJoint, soma:'hasJointState', DoorJointState)),
    tell(triple(DoorJointState, soma:'hasJointPosition', 0.0)).


create_door_hinge(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, HingeJointName),
    urdf_joint_parent_link(URDF, HingeJointName, HingeLink),
    tell(has_type(Hinge, hsr_rooms:'Hinge')),
    tell(triple(Hinge, urdf:'hasURDFName', HingeLink)),
    %tell(has_type(HingeJoint, urdf:'HingeJoint')),
    %tell(triple(HingeJoint, urdf:'hasURDFName', HingeJointName)),
    tell(triple(Door, knowrob:'doorHingedTo', Hinge)).


create_door_handles(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_child_joints(URDF, DoorLink, ChildJoints),
    forall(member(ChildJoint, ChildJoints),
    (
        urdf_joint_child_link(URDF, ChildJoint, DoorHandleLink),
        tell(has_type(DoorHandle, hsr_rooms:'DoorHandle')),
        tell(triple(DoorHandle, urdf:'hasURDFName', DoorHandleLink)),
        (sub_string(DoorHandleLink,_,_,_,inside)
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


is_door(Door) :-
    has_type(Door, hsr_rooms:'Door').


create_passage(Passage, PassageLink) :-
    tell(has_type(Passage, hsr_rooms:'Passage')),
    tell(triple(Passage, urdf:'hasURDFName', PassageLink)).


is_passage(Passage) :-
    has_type(Passage, hsr_rooms:'Passage').


assign_connecting_rooms(RoomLinkage, RoomLinkageLink) :-
    split_string(RoomLinkageLink, ":", "", [ExpRoom1Link, ExpRoom2Link, _]),
    has_type(Room1, hsr_rooms:'Room'),
    has_type(Room2, hsr_rooms:'Room'),
    urdf_room_center_link(Room1, ActRoom1Link),
    urdf_room_center_link(Room2, ActRoom2Link),
    sub_string(ActRoom1Link,_,_,_,ExpRoom1Link),
    sub_string(ActRoom2Link,_,_,_,ExpRoom2Link),
    tell(has_type(Location, soma:'Location')),
    tell(triple(Location, soma:'isLinkedTo', Room1)),
    tell(triple(Location, soma:'isLinkedTo', Room2)),
    tell(triple(RoomLinkage, dul:'hasLocation', Location)).


assign_connecting_rooms(RoomLinkage, RoomLinkageLink) :-
    split_string(RoomLinkageLink, ":", "", [ExpRoom1Link, ExpRoom2Link, _]),
    sub_string(ExpRoom1Link,_,_,_,"outside"),
    has_type(Room, hsr_rooms:'Room'),
    urdf_room_center_link(Room, ActRoomLink),
    sub_string(ActRoomLink,_,_,_,ExpRoom2Link),
    has_type(Outside, hsr_rooms:'Outside'),
    tell(has_type(Location, soma:'Location')),
    tell(triple(Location, soma:'isLinkedTo', Outside)),
    tell(triple(Location, soma:'isLinkedTo', Room)),
    tell(triple(RoomLinkage, dul:'hasLocation', Location)).


assign_connecting_rooms(RoomLinkage, RoomLinkageLink) :-
    split_string(RoomLinkageLink, ":", "", [ExpRoom1Link, ExpRoom2Link, _]),
    sub_string(ExpRoom2Link,_,_,_,"outside"),
    has_type(Room, hsr_rooms:'Room'),
    urdf_room_center_link(Room, ActRoomLink),
    sub_string(ActRoomLink,_,_,_,ExpRoom1Link),
    has_type(Outside, hsr_rooms:'Outside'),
    tell(has_type(Location, soma:'Location')),
    tell(triple(Location, soma:'isLinkedTo', Outside)),
    tell(triple(Location, soma:'isLinkedTo', Room)),
    tell(triple(RoomLinkage, dul:'hasLocation', Location)).


init_door_paths :-
    findall([OriginLinkage, DestinationLinkage],
    (   
        has_type(OriginLinkage, hsr_rooms:'RoomLinkage'),
        has_type(DestinationLinkage, hsr_rooms:'RoomLinkage'),
        triple(OriginLinkage, dul:'hasLocation', OriginLocation),
        triple(DestinationLinkage, dul:'hasLocation', DestinationLocation),
        not same_as(OriginLocation, DestinationLocation),
        triple(OriginLocation, soma:'isLinkedTo', Room),
        triple(DestinationLocation, soma:'isLinkedTo', Room)
        
    ), 
    Paths),
    forall(member([Origin, Destination], Paths),
    (
        tell(has_type(Path, hsr_rooms:'Path')),
        tell(triple(Path, hsr_rooms:'hasOrigin', Origin)),
        tell(triple(Path, hsr_rooms:'hasDestination', Destination)),
        assign_path_costs(Path, Origin, Destination)
    )).


assign_path_costs(Path, Origin, Destination) :-
    has_urdf_name(Origin, OriginURDFName),
    urdf_frame_add_prefix_(OriginURDFName, OriginLink),
    has_urdf_name(Destination, DestinationURDFName),
    urdf_frame_add_prefix_(DestinationURDFName, DestinationLink),
    get_urdf_origin(Map),
    tf_lookup_transform(Map, OriginLink, pose(OriginPosition, _)),
    tf_lookup_transform(Map, DestinationLink, pose(DestinationPosition, _)),
    euclidean_distance(OriginPosition, DestinationPosition, Distance),
    robot_velocity(Velocity),
    NeededTime is Distance/Velocity,
    ((has_type(Destination, hsr_rooms:'Door'), get_door_state(Destination, 0))
    ->  
    (
        door_opening_time(OpeningTime),
        FinalCosts is NeededTime + OpeningTime,
        update(triple(Path, hsr_rooms:'hasCosts', FinalCosts))

    );
    (
        FinalCosts is NeededTime,
        update(triple(Path, hsr_rooms:'hasCosts', FinalCosts))
    )).



shortest_path_between_rooms(OriginRoom, DestinationRoom, Path) :-
    robot_velocity(Velocity),
    door_opening_time(OpeningTime),
    findall([OriginLinkage, DestinationLinkage],
    (
        triple(OriginLocation, soma:'isLinkedTo', OriginRoom),
        triple(DestinationLocation, soma:'isLinkedTo', DestinationRoom),
        triple(OriginLinkage, dul:'hasLocation', OriginLocation),
        triple(DestinationLinkage, dul:'hasLocation', DestinationLocation)
    ),
    OriginDestinationPairs),
    findall([Costs, PossiblePath], 
    (
        member([Origin, Destination], OriginDestinationPairs),
        get_urdf_origin(Map),
        has_urdf_name(Origin, OriginLink),
        tf_lookup_transform(Map, OriginLink, pose(OriginPosition, _)),
        tf_lookup_transform(Map, 'base_footprint', pose(RobotPosition, _)),
        euclidean_distance(OriginPosition, RobotPosition, InitialDistance),
        ((has_type(Origin, hsr_rooms:'Door'), get_door_state(Origin, 0))
        -> InitialCosts is InitialDistance/Velocity + OpeningTime
        ; InitialCosts is InitialDistance/Velocity),
        a_star(Origin, InitialDistance, Destination, PossiblePath, Costs)
    ), 
    PossiblePaths),
    min_member([_, Path], PossiblePaths).



perceiving_pose_of_door(Door, Pose, DoorHandle) :-
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
        Pose = [[NewX, NewY, 0.0], Rotation],
        inside_door_handle(Door, DoorHandle)
    );
    (
        DeltaY is Offset,
        Angle is pi,
        angle_to_quaternion(Angle, DeltaRotation),
        tf_transform_pose(DoorLink, Origin, pose([DeltaX, DeltaY, 0.0], DeltaRotation), pose([NewX, NewY, _], Rotation)),
        Pose = [[NewX, NewY, 0.0], Rotation],
        outside_door_handle(Door, DoorHandle)
    )).


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


update_door_paths(Door) :-
    findall(Path, 
    (
        (triple(Path, hsr_rooms:'hasOrigin', Door);
        triple(Path, hsr_rooms:'hasDestination', Door))
    ),
    Paths),
    forall(member(Path, Paths),
    (
        triple(Path, hsr_rooms:'hasOrigin', Origin),
        triple(Path, hsr_rooms:'hasDestination', Destination),
        assign_path_costs(Path, Origin, Destination)
    )).
    

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


get_angle_to_open_door(Door, Angle) :-
    get_urdf_id(URDF),
    door_joint(Door, DoorJoint),
    has_urdf_name(DoorJoint, DoorJointName),
    urdf_joint_hard_limits(URDF, DoorJointName, [_, UpperLimit], _, _),
    triple(DoorJoint, soma:'hasJointState', DoorJointState),
    triple(DoorJointState, soma:'hasJointPosition', JointPosition),
    Angle is UpperLimit - JointPosition.



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
    door_joint(Door, DoorJoint),
    triple(DoorJoint, soma:'hasJointState', DoorJointState),
    triple(DoorJointState, soma:'hasJointPosition', DoorJointPosition),
    min_door_joint_angle(MaxAngle),
    MinAngle is -1*MaxAngle,
    ( DoorJointPosition < MaxAngle, DoorJointPosition > MinAngle 
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


door_joint(Door, DoorJoint) :-
    has_urdf_name(Door, DoorLink),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, DoorJointName),
    has_urdf_name(DoorJoint, DoorJointName).


door_hinge(Door, Hinge) :-
    triple(Door, knowrob:'doorHingedTo', Hinge).

inside_door_handle(Door, DoorHandle) :-
    triple(Location, soma:'isInsideOf', Door),
    once(has_location(DoorHandle, Location)).

outside_door_handle(Door, DoorHandle) :-
    triple(Location, hsr_rooms:'isOutsideOf', Door),
    once(has_location(DoorHandle, Location)).


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


%% min_door_joint_angle(Angle)
%
%  returns the the minimum opening angle
%  of a door to be seen as open
%
%  @param Angle Minimum opening angle
min_door_joint_angle(Angle) :-
    Angle = 1.22. % corresponds to 70 degree


robot_velocity(Velocity) :-
    Velocity = 0.15. % Robot moves about 0.15 meters per second


door_opening_time(Time) :-
    Time is 60.  % Robot needs about 60 seconds to open a door
