:- module(rooms,
    [
        init_rooms/0,
        connect_rooms/0,
        all_rooms_of_type/2,
        room_corner_point_positions/2,
        all_rooms/1,
        is_room/1
    ]).


:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).


:- use_module('./doors', [is_room_linkage/1]).


%% is_room(?Room) is nondet
%
% True if Room is an instance of type hsr_rooms:'Rooms'
%
% @param Room A room IRI 
%
is_room(Room) :-
    has_type(Room, hsr_rooms:'Room').


%% all_rooms(?Rooms) is det
%
% Returns all instances of type hsr_rooms:'Room'
%
% @param Rooms A list of room IRIs 
%
all_rooms(Rooms) :-
    findall(Room, has_type(Room, hsr_rooms:'Room'), Rooms).


%% all_rooms_of_type(RoomType, ?Rooms) is det
%
% Returns all instances of type RoomType
%
% @param RoomType A room class IRI, Rooms A list of room IRIs
%
all_rooms_of_type(RoomType, Rooms) :-
    findall(Room, has_type(Room, RoomType), Rooms).



%% urdf_room_center_link(Room, ?RoomLink)
%
% True if RoomLink is the center room link of Room
%
% @param Room A room IRI, RoomLink URDF link name as string
%
urdf_room_center_link(Room, RoomLink) :-
    triple(CenterPointLocation, knowrob:'isInCenterOf', Room),
    has_location(CenterPoint, CenterPointLocation),
    triple(CenterPoint, urdf:'hasURDFName', RoomLink).


%% init_rooms is det
%
% Reads all room links from the URDF and creates 
% an instance of type hsr_rooms:'Room' for each
%
init_rooms :-
    init_outside,
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    findall(RoomLink,
    (
        member(RoomLink, Links),
        sub_string(RoomLink,_,_,_,"room_center_link")
    ), 
    RoomLinks),
    forall(member(RoomLink2, RoomLinks), init_room(RoomLink2)).


%% init_room is nondet
%
% Creates a single instance of type hsr_rooms:'Room' and 
% assigns room points and surfaces
%
% @param RoomLink URDF link name as string
%
init_room(RoomLink) :-
    create_room(RoomLink, Room),
    assign_room_points(Room, RoomLink),
    assign_room_surfaces(Room).


init_outside :-
    tell(has_type(Outside, hsr_rooms:'Outside')),
    assign_room_surfaces(Outside).


create_room(RoomLink, Room) :-
    sub_string(RoomLink,_,_,_,"kitchen"),
    tell(has_type(Room, hsr_rooms:'Kitchen')),
    !.

create_room(RoomLink, Room) :-
    sub_string(RoomLink,_,_,_,"living_room"),
    tell(has_type(Room, hsr_rooms:'LivingRoom')),
    !.

create_room(RoomLink, Room) :-
    (sub_string(RoomLink,_,_,_,"sleeping_room");
    sub_string(RoomLink,_,_,_,"bedroom")),
    tell(has_type(Room, hsr_rooms:'SleepingRoom')),
    !.

create_room(RoomLink, Room) :-
    sub_string(RoomLink,_,_,_,"office"),
    tell(has_type(Room, hsr_rooms:'Office')),
    !.

create_room(RoomLink, Room) :-
    sub_string(RoomLink,_,_,_,"dining_room"),
    tell(has_type(Room, hsr_rooms:'DiningRoom')),
    !.

create_room(RoomLink, Room) :-
    sub_string(RoomLink,_,_,_,"hall"),
    tell(has_type(Room, hsr_rooms:'Hall')),
    !.


assign_room_points(Room, RoomLink) :-
    assign_room_center_point(Room, RoomLink),
    assign_room_corner_points(Room, RoomLink).


assign_room_center_point(Room, RoomLink) :-
    tell(has_type(CenterPoint, knowrob:'Point')),
    tell(has_urdf_name(CenterPoint, RoomLink)),
    tell(has_type(CenterPointLocation, soma:'Location')),
    tell(has_location(CenterPoint, CenterPointLocation)),
    tell(triple(CenterPointLocation, knowrob:'isInCenterOf', Room)).


assign_room_corner_points(Room, RoomLink) :-
    tell(has_type(CornerPointCollection, dul:'Collection')),
    get_urdf_id(URDF),
    urdf_link_child_joints(URDF, RoomLink, CornerJoints),
    forall(
    (
        member(CornerJoint, CornerJoints),
        urdf_joint_child_link(URDF, CornerJoint, CornerLink)
    ), 
    (
        tell(has_type(CornerPoint, knowrob:'Point')),
        tell(has_urdf_name(CornerPoint, CornerLink)),
        tell(triple(CornerPoint, dul:'isMemberOf', CornerPointCollection))
    )),
    tell(has_type(CornerPointLocation, soma:'Location')),
    tell(has_location(CornerPointCollection, CornerPointLocation)),
    tell(triple(CornerPointLocation, hsr_rooms:'isInCornerOf', Room)).


assign_room_surfaces(Room) :-
    tell(has_type(Floor, hsr_rooms:'Floor')),
    tell(triple(Room, hsr_rooms:'hasSurface', Floor)).


room_center_point_position(Room, RoomLink, [X, Y, Z]) :-
    triple(CenterPointLocation, knowrob:'isInCenterOf', Room),
    has_location(CenterPoint, CenterPointLocation),
    urdf_tf_frame(CenterPoint, RoomLink),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, RoomLink, pose([X, Y, Z], _)).


room_corner_point_positions(Room, Positions) :-
    once((
        triple(CornerPointLocation, hsr_rooms:'isInCornerOf', Room),
        has_location(CornerPointCollection, CornerPointLocation),
        get_urdf_origin(Origin),
        findall(Position, 
        (
            triple(CornerPoint, dul:'isMemberOf', CornerPointCollection),
            urdf_tf_frame(CornerPoint, CornerLink),
            tf_lookup_transform(Origin, CornerLink, pose(Position, _))
        ), 
        Positions)
    )).


connect_rooms :-
    forall((
        is_room_linkage(RoomLinkage),
        has_urdf_name(RoomLinkage, RoomLinkageLink)
    ), 
    assign_connecting_rooms(RoomLinkage, RoomLinkageLink)).


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










    
