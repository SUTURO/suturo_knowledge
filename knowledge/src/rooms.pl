:- module(rooms,
    [
        init_rooms/0,
        all_rooms_of_type/2,
        room_center_point_position/3,
        room_corner_point_positions/2,
        urdf_room_center_link/2,
        all_rooms/1,
        is_room/1
    ]).


:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).



init_rooms :-
    tell(has_type(Outside, hsr_rooms:'Outside')),
    assign_room_surfaces(Outside),
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    findall(RoomLink,
    (
        member(RoomLink, Links),
        sub_string(RoomLink,_,_,_,"room_center_link")
    ), 
    RoomLinks),
    forall(member(RoomLink2, RoomLinks),
    (
        create_room(RoomLink2, Room),
        assign_room_points(Room, RoomLink2),
        assign_room_surfaces(Room)
    )).


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
    tell(has_type(CenterPoint, knowrob:'Point')),
    tell(triple(CenterPoint, urdf:'hasURDFName', RoomLink)),
    tell(has_type(CenterPointLocation, soma:'Location')),
    tell(triple(CenterPoint, dul:'hasLocation', CenterPointLocation)),
    tell(triple(CenterPointLocation, knowrob:'isInCenterOf', Room)),

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
        tell(triple(CornerPoint, urdf:'hasURDFName', CornerLink)),
        tell(triple(CornerPoint, dul:'isMemberOf', CornerPointCollection))
    )),
    tell(has_type(CornerPointLocation, soma:'Location')),
    tell(triple(CornerPointCollection, dul:'hasLocation', CornerPointLocation)),
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

urdf_room_center_link(Room, RoomLink) :-
    triple(CenterPointLocation, knowrob:'isInCenterOf', Room),
    has_location(CenterPoint, CenterPointLocation),
    triple(CenterPoint, urdf:'hasURDFName', RoomLink).


is_room(Room) :-
    has_type(Room, hsr_rooms:'Room').

all_rooms(Rooms) :-
    findall(Room, has_type(Room, hsr_rooms:'Room'), Rooms).

all_rooms_of_type(RoomType, Rooms) :-
    findall(Room, has_type(Room, RoomType), Rooms).








    
