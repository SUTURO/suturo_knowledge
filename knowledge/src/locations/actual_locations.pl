:- module(actual_locations,
    [
        is_legal_obj_position/1,
        place_objects/0,
        object_at_location/4,
        forget_object_at_location/1,
        surfaces_not_visited_in_room/2,
        robot_in_room/1,
        furniture_in_room/2,
        pose_is_outside/1,
        object_supported_by_surface/2,
        objects_supported_by_surface/2,
        objects_supported_by_surfaces/2,
        objects_in_room/2
    ]).


:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).


:- use_module(library('model/environment/rooms'), 
    [
        is_room/1,
        room_corner_point_positions/2
    ]).

:- use_module(library('model/environment/surfaces'), 
    [
        is_surface/1,
        surfaces_not_visited/1
    ]).

:- use_module(library('model/environment/furnitures'), 
    [
        has_surface/2,
        all_furnitures/1,
        is_furniture/1
    ]).

:- use_module(library('model/objects/object_info'), 
    [
        is_suturo_object/1,
        hsr_existing_objects/1
    ]).
:- use_module(library('locations/spatial_comp'), [surface_dimensions/4]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% actual object locations %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% is_legal_obj_position(?Position)
%
% True if position is on any furniture surface or on the floor
%
% @param Position A Position as [X, Y, Z]
%
is_legal_obj_position([X, Y, Z]) :-
    is_surface(Surface),
    position_supported_by_surface([X, Y, Z], Surface).


%% place_objects is det
%
% Assigns each known object its current location
%
place_objects :-
    hsr_existing_objects(Objects),
    forall(member(Object, Objects), 
    (
        ignore(object_at_location(Object, _, _, _))
    )).


%% object_at_location(?Object, ?Room, ?Furniture, ?Surface) is nondet
%
% True if Object is located in room Room, on furniture 
% Furniture and supported by surface Surface
%
% @param Object An object IRI 
% @param Room A room IRI 
% @param Furniture A furniture IRI 
% @param Surface A surface IRI 
%
object_at_location(Object, Room, Furniture, Surface) :-
    is_suturo_object(Object),
    once(has_location(Object, _)),
    !,
    object_in_room(Object, Room),
    object_on_furniture(Object, Furniture),
    once(object_supported_by_surface(Object, Surface)),
    !.


%% forget_object_at_location(?Object) is nondet
%
% Removes the actual location of Object from the knowledge base
%
% @param Object, object instance
%
forget_object_at_location(Object) :-
    (
        has_location(Object, ObjectLocation),
        forall(triple(ObjectLocation, knowrob:'isInsideOf', _), 
            tripledb_forget(ObjectLocation, knowrob:'isInsideOf', _)),
        forall(triple(ObjectLocation, knowrob:'isOntopOf', _), 
            tripledb_forget(ObjectLocation, knowrob:'isOntopOf', _)),
        forall(triple(ObjectLocation, soma:'isSupportedBy', _), 
            tripledb_forget(ObjectLocation, soma:'isSupportedBy', _))
    );
    not has_location(Object, _).


%% object_in_room(?Object, ?Room) is nondet
%
% True if Object is located in Room
%
% @param Object, object instance
% @param Room, room instance
%
object_in_room(Object, Room) :-
    once(has_location(Object, ObjectLocation)),
    triple(ObjectLocation, knowrob:'isInsideOf', Room),
    is_room(Room).


object_in_room(Object, Room) :-
    object_tf_frame(Object, ObjectFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, ObjectFrame, pose(ObjectPosition, _)),
    position_in_room(ObjectPosition, Room),
    once(has_location(Object, ObjectLocation)),
    tell(triple(ObjectLocation, knowrob:'isInsideOf', Room)).


%% objects_in_room(?Room, ?Objects) is det
%
% Returns all objects located in Room
%
% @param Room, room instance
% @param Objects, list of object instances
%
objects_in_room(Room, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_in_room(Object, Room)
    ),
    Objects).


%% object_on_furniture(?Object, ?Furniture) is nondet
%
% True if Object is located on Fruniture
%
% @param Object, object instance
% @param Furniture, furniture instance
%
object_on_furniture(Object, Furniture) :-
    once(has_location(Object, ObjectLocation)),
    triple(ObjectLocation, knowrob:'isInsideOf', Furniture),
    is_furniture(Furniture).


object_on_furniture(Object, Furniture) :-
    once(has_location(Object, ObjectLocation)),
    triple(ObjectLocation, knowrob:'isOnTopOf', Furniture),
    is_furniture(Furniture).


object_on_furniture(Object, Furniture) :-
    has_surface(Furniture, Surface),
    object_supported_by_surface(Object, Surface).


%% objects_on_furniture(?Furniture, ?Objects) is det
%
% Returns all objects located on Furniture
%
% @param Furniture, furniture instance
% @param Objects, list of object instances
%
objects_on_furniture(Furniture, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_on_furniture(Object, Furniture)
    ),
    Objects).



%% objects_supported_by_surface(?Object, ?Surface) is nondet
%
% True if Object is located on Surface
%
% @param Object, object instance
% @param Surface, surface instance
%
object_supported_by_surface(Object, Surface) :-
    once(has_location(Object, ObjectLocation)),
    triple(ObjectLocation, soma:'isSupportedBy', Surface),
    !.


object_supported_by_surface(Object, Surface) :-
    once(has_location(Object, ObjectLocation)),
    object_tf_frame(Object, ObjectFrame),
    tf_lookup_transform(map, ObjectFrame, pose(Position, _)),
    position_supported_by_surface(Position, Surface),
    tell(triple(ObjectLocation, soma:'isSupportedBy', Surface)).



%% objects_supported_by_surface(?Surface, ?Objects) is det
%
% returns all objects located on Surface
%
% @param Surface, surface instance
% @param Objects, list of object instances
%
objects_supported_by_surface(Surface, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_supported_by_surface(Object, Surface)
    ),
    Objects).


%% objects_supported_by_surfaces(?Surfaces, ?Objects) is det
%
% returns all objects located on the list of Surfaces
%
% @param Surfaces, list of surface instances
% @param Objects, list of object instances
%
objects_supported_by_surfaces(Surfaces, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        member(Surface, Surfaces),
        object_supported_by_surface(Object, Surface)
    ),
    Objects).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% actual furniture location %%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% furniture_in_room(?Furniture, ?Room) is nondet
%
% True if Furniture is located in Room
%
% @param Furniture, furniture instance
% @param Room, room instance
%
furniture_in_room(Furniture, Room) :-
    once(has_location(Furniture, FurnitureLocation)),
    triple(FurnitureLocation, knowrob:'isInsideOf', Room),
    !.


furniture_in_room(Furniture, Room) :-
    once(has_location(Furniture, FurnitureLocation)),
    urdf_tf_frame(Furniture, FurnitureFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, FurnitureFrame, pose(ObjectPosition, _)),
    position_in_room(ObjectPosition, Room),
    tell(triple(FurnitureLocation, knowrob:'isInsideOf', Room)),
    !.


%% furnitures_in_room(?Room, ?Furnitures) is det
%
% returns all furnitures located in Room
%
% @param Furnitures, list of furniture instances
% @param Room, room instance
%
furnitures_in_room(Room, Furnitures) :-
    all_furnitures(AllFurnitures),
    findall(Furniture, 
    (
        member(Furniture, AllFurnitures),
        furniture_in_room(Furniture, Room)
    ),
    Furnitures).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% actual surface location %%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% surface_in_room(?Surface, ?Room) is nondet
%
% True if Surface is located in Room
%
% @param Surface, surface instance
% @param Room, room instance
%
surface_in_room(Surface, Room) :-
    has_surface(Furniture, Surface),
    furniture_in_room(Furniture, Room).


%% surfaces_in_room(?Room, ?Surfaces) is det
%
% returns all surfaces located in Room
%
% @param Surfaces, list of surface instances
% @param Room, room instance
%
surfaces_in_room(Room, Surfaces) :-
    furnitures_in_room(Room, Furnitures),
    findall(Surface, 
    (
        member(Furniture, Furnitures),
        has_surface(Furniture, Surface)
    ),
    Surfaces).


%% surfaces_not_visited_in_room(?Room, ?Surfaces) is det
%
% returns all surfaces not visited located in Room 
%
% @param Surfaces, list of surface instances
% @param Room, room instance
%
surfaces_not_visited_in_room(Room, Surfaces) :-
    is_room(Room),
    surfaces_not_visited(SurfacesNotVisited),
    surfaces_in_room(Room, SurfacesInRoom),
    findall(Surface, 
    (
        member(Surface, SurfacesNotVisited),
        member(Surface, SurfacesInRoom)
    ), 
    Surfaces).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% actual robot location %%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% robot_in_room(?Room) is nondet
%
% True if robot is located in Room
%
% @param Room, room instance
%
robot_in_room(Room) :-
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, 'base_footprint', pose(RobotPosition, _)),
    position_in_room(RobotPosition, Room),
    !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% general %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% position_in_room(?Position, ?Room) is nondet
%
% True if  Position is located in Room
%
% @param Room, room instance
% @param Postion, point as cartesian coordinates
%
position_in_room(Position, Room) :-
    is_room(Room),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(Position, CornerPoints),
    !.


position_in_room(_, Room) :-
    has_type(Room, hsr_rooms:'Outside'),
    !.


%% pose_is_outside(?Position) is nondet
%
% True if Position is outside
%
% @param Position, point as cartesian coordinates
%
pose_is_outside(Position) :-
    position_in_room(Position, Room),
    has_type(Room, hsr_rooms:'Outside').


%% position_supported_by_surface(?Position, ?Surface) is nondet
%
% True if Position is located on Surface
%
% @param Position, point as cartesian coordinates
% @param Surface, surface instance
%
% Position is relative to map
position_supported_by_surface(Position, Surface) :-
    is_surface(Surface),
    surface_dimensions(Surface, Depth, Width, _),
    threshold_surface(ThAbove, ThBelow),
    urdf_tf_frame(Surface, SurfaceFrame),
    tf_transform_point(map, SurfaceFrame, Position, [X, Y, Z]),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    Depth/2 >= abs(X),
    !.


position_supported_by_surface([X,Y,Z], Surface) :-
    ZLimit is 0.28,
    Z =< ZLimit,
    position_in_room([X,Y,Z], Room),
    has_surface(Room, Surface),
    !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%% Point in Polygon Test %%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%% point_in_polygon(?Q, ?PolygonCornerPoints) is nondet
%
% True id Q is located inside the area defined by PolygonCornerPoints
%
% @param Q, point as cartesian coordinates
% @param PolygonCornerPoints, lits of points as cartesian coordinates
%
point_in_polygon(Q, PolygonCornerPoints) :-
    nth0(0, PolygonCornerPoints, First),
    InitialSign is -1,
    next_edge(Q, First, PolygonCornerPoints, InitialSign, Sign),
    (Sign == 1; Sign == 0).

next_edge(Q, R, PolygonCornerPoints, CurrentSign, Sign) :-
    not nextto(R, _, PolygonCornerPoints),
    nth0(0, PolygonCornerPoints, First),
    right_cross(Q, R, First, Result),
    Sign is CurrentSign * Result.

next_edge(Q, R, PolygonCornerPoints, CurrentSign, Sign) :-
    nextto(R, S, PolygonCornerPoints),
    right_cross(Q, R, S, Result),
    NewSign is CurrentSign * Result,
    next_edge(Q, S, PolygonCornerPoints, NewSign, Sign).

right_cross([QX, QY, _], [RX, RY, _], [SX, SY, _], Result) :-
    (RY < SY; RY = SY),
    (QY > SY; QY < RY; QY = RY),
    Result is 1.

right_cross([QX, QY, _], [RX, RY, _], [SX, SY, _], Result) :-
    (RY < SY; RY = SY),
    not (QY > SY; QY < RY; QY = RY),
    det([QX, QY, _], [RX, RY, _], [SX, SY, _], Det),
    signum_function(Det, Result).

right_cross([QX, QY, _], [RX, RY, _], [SX, SY, _], Result) :-
    RY > SY,
    (QY < SY; QY = SY; QY > RY),
    Result is 1.

right_cross([QX, QY, _], [RX, RY, _], [SX, SY, _], Result) :-
    RY > SY,
    not (QY < SY; QY = SY; QY > RY),
    det([QX, QY, _], [SX, SY, _], [RX, RY, _], Det),
    signum_function(Det, Result).

det([QX, QY, _], [RX, RY, _], [SX, SY, _], Result) :-
    Result is (RX - QX)*(SY - QY) - (RY - QY)*(SX - QX).

signum_function(X, Y) :-
    (X > 0; X = 0),
    Y is 1.

signum_function(X, Y) :-
    X < 0,
    Y is -1.
