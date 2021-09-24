:- module(doors,
    [
        init_doors/0,
        init_door_paths/0,
        is_door/1,
        is_passage/1,
        inside_door_handle/2,
        outside_door_handle/2,
        get_all_door_states/1,
        get_door_state/2,
        update_door_paths/1,
        door_handles/2
    ]).


:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).

:- use_module(library('locations/spatial_comp'), [euclidean_distance/3]).

is_room_linkage(RoomLinkage) :-
    has_type(RoomLinkage, hsr_rooms:'RoomLinkage').

is_door(Door) :-
    has_type(Door, hsr_rooms:'Door').


is_passage(Passage) :-
    has_type(Passage, hsr_rooms:'Passage').


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


%% init_doors is nondet
%  
%  Creates a room linkage instance
%  for each door and passage specified in the URDF
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
    init_door(Link)),
    forall((
        member(Link, Links),
        sub_string(Link,_,_,_,path)
    ),
    create_passage(_, Link)).


%% init_door(?DoorLink) is nondet
%
% Initializes a single instance of
% concept Door and creates door joint 
% door hinge and door handles
%
% @param DoorLink, string of corresponding URDF link name
%
init_door(DoorLink) :-
    create_door(DoorLink, Door),
    create_door_joint(DoorLink),
    create_door_hinge(Door, DoorLink),
    create_door_handles(Door, DoorLink).


%% create_door(?DoorLink, ?Door) is nondet
%
% Creates a single instance of
% concept Door
%
% @param DoorLink, string of corresponding URDF link name
% @param Door, created door instance
%
create_door(DoorLink, Door) :-
    tell(has_type(Door, hsr_rooms:'Door')),
    tell(has_urdf_name(Door, DoorLink)).


%% create_door_joint(?DoorLink) is nondet
%
% Creates the door joint as an instance of 
% concept soma:'Joint'
%
% @param DoorLink, string of corresponding URDF link name
%
create_door_joint(DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, DoorJointName),
    tell(has_type(DoorJoint, soma:'Joint')),
    tell(has_urdf_name(DoorJoint, DoorJointName)),
    tell(has_type(DoorJointState, soma:'JointState')),
    tell(triple(DoorJoint, soma:'hasJointState', DoorJointState)),
    tell(triple(DoorJointState, soma:'hasJointPosition', 0.0)).


%% create_door_hinge(?Door, ?DoorLink) is nondet
%
% Creates the door hinge as an instance of 
% concept hsr_rooms:'Hinge'
%
% @param Door, door instance
% @param DoorLink, string of corresponding URDF link name
%
create_door_hinge(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, HingeJointName),
    urdf_joint_parent_link(URDF, HingeJointName, HingeLink),
    tell(has_type(Hinge, hsr_rooms:'Hinge')),
    tell(has_urdf_name(Hinge, HingeLink)),
    tell(triple(Door, knowrob:'doorHingedTo', Hinge)).


%% create_door_handles(?Door, ?DoorLink) is nondet
%
% Creates the door handles as instances of 
% concept hsr_rooms:'DoorHandle'
%
% @param Door, door instance
% @param DoorLink, string of corresponding URDF link name
%
create_door_handles(Door, DoorLink) :-
    get_urdf_id(URDF),
    urdf_link_child_joints(URDF, DoorLink, ChildJoints),
    forall(member(ChildJoint, ChildJoints),
    (
        urdf_joint_child_link(URDF, ChildJoint, DoorHandleLink),
        tell(has_type(DoorHandle, hsr_rooms:'DoorHandle')),
        tell(has_urdf_name(DoorHandle, DoorHandleLink)),
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


%% create_passage(?Passagem ?PassageLink) is nondet
%
% Creates an instance of concept hsr_rooms:'Passage'
%
% @param Passage, the created passage instance
% @param PassageLink, the URDF link name
%
create_passage(Passage, PassageLink) :-
    tell(has_type(Passage, hsr_rooms:'Passage')),
    tell(has_urdf_name(Passage, PassageLink)).


%% init_door_paths is det 
%
% Creates an instance of the concept hsr_rooms:'Path'
% for all pairs of room linkages linked to the same room
%
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


%% assign_path_costs(?Path, ?Origin, ?Destination) is nondet
%
% Calculates the distance between Origin and Destination and 
% assigns the calculated distance to Path as path costs
%
% @param Path, path instance
% @param Origin, room linkage instance
% @param Destination, room linkage instance
%
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


%% update_door_paths(?Door) is nondet
%
% Recalculates path costs for all path instances
% associated with the given room linkage instance 
%
% @param Door, a room linkage instance
%
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


%% door_joint(?Door, ?DoorJoint) is nondet
%
% Returns the door joint of the given door instance
%
% @param Door, door instance
% @param DoorJoint, door joint instance
%
door_joint(Door, DoorJoint) :-
    has_urdf_name(Door, DoorLink),
    get_urdf_id(URDF),
    urdf_link_parent_joint(URDF, DoorLink, DoorJointName),
    has_urdf_name(DoorJoint, DoorJointName).


%% door_hinge(?Door, ?Hinge) is nondet
%
% Returns the hinge of the given door instance
%
% @param Door, door instance
% @param Hinge, hinge instance
%
door_hinge(Door, Hinge) :-
    triple(Door, knowrob:'doorHingedTo', Hinge).


%% door_handles(?Door, ?DoorHandles) is nondet
%
% Returns the door handles of the given door
%
% @param Door, door instance
% @param DoorHandles, list of door handle instances
%
door_handles(Door, DoorHandles) :-
    findall(DoorHandle,
    (
        inside_door_handle(Door, DoorHandle);
        outside_door_handle(Door, DoorHandle)
    ), 
    DoorHandles).


%% inside_door_handle(?Door, ?DoorHandle) is nondet
%
% Returns the inside door handle of the given door
%
% @param Door, door instance
% @param DoorHandle, door handle instance
%
inside_door_handle(Door, DoorHandle) :-
    triple(Location, soma:'isInsideOf', Door),
    once(has_location(DoorHandle, Location)).


%% outside_door_handle(?Door, ?DoorHandle) is nondet
%
% Returns the outside door handle of the given door
%
% @param Door, door instance
% @param DoorHandle, door handle instance
%
outside_door_handle(Door, DoorHandle) :-
    triple(Location, hsr_rooms:'isOutsideOf', Door),
    once(has_location(DoorHandle, Location)).


%% min_door_joint_angle(Angle)
%
%  returns the the minimum opening angle
%  of a door to be seen as open
%
%  @param Angle Minimum opening angle
min_door_joint_angle(Angle) :-
    Angle = 1.22. % corresponds to 70 degree
