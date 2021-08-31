:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3, % deprecated
        hsr_existing_object_at/2,
        hsr_existing_object_at_thr/2,
        hsr_existing_object_at_thr/3,
	    object_pose/2,
        surface_pose_in_map/2,
        object_supported_by_surface/2,
        position_supported_by_surface/2,
        distance_to_robot/2,
        %Debug
        is_legal_obj_position/1,
        place_objects/0,
        object_in_room/2,
        surface_in_room/2,
        furniture_in_room/2,
        robot_in_room/1,
        object_at_location/4,
        forget_object_at_location/1,
        object_at_predefined_location/3,
        object_in_predefined_room/2,
        object_on_predefined_furniture/2,
        object_on_furniture/2,
        misplaced_objects_at_predefined_location/3,
        surface_at_predefined_location/3,
        surfaces_at_predefined_location/3,
        objects_supported_by_surfaces/2,
        object_supported_by_surface/2,
        furnitures_in_room/2,
        surfaces_in_room/2,
        objects_in_room/2,
        has_predefined_location/2,
        objects_supported_by_surface/2,
        locations_not_visited/1,
        locations_not_visited/2,
        object_in_room/2,
        is_misplaced/1,
        position_in_room/2,
        pose_is_outside/1
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).

:- rdf_meta
    hsr_lookup_transform(r,r,?,?),
    hsr_existing_object_at(r,r,?).

hsr_lookup_transform(SourceFrame, TargetFrame, Translation, Rotation) :-
    tf_lookup_transform(SourceFrame, TargetFrame, pose(Translation,Rotation)).

% deprecated. Use hsr_existing_object_at/2.
hsr_existing_object_at(Pose, Threshold, Instance) :-
    has_type(Instance, owl:'NamedIndividual'),
    instance_of(Instance, dul:'PhysicalObject'),
    triple(Instance, hsr_objects:'supportable', true),
    %object_pose(Instance, OldPose),
    is_at(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).

hsr_existing_object_at([X,Y,Z], Instance) :-
    hsr_existing_object_at_thr([X,Y,Z], 0, Instance).

hsr_existing_object_at([map,_,Pos, _], Instance) :-
    hsr_existing_object_at(Pos, Instance).

hsr_existing_object_at_thr([X,Y,Z], Threshold) :-
    hsr_existing_object_at_thr([X,Y,Z], Threshold, _).

% to do:
% 1. we need a variation where the input is the Size of another object that should be placed.
% Right now we are only computing edge of one obj to center of another.
% 2. This is based on the Object dimensions. It should be using the Group dimensions
% just like object_goal_pose in assignplaces.pl
hsr_existing_object_at_thr([X,Y,Z], Threshold1, Instance) :- 
    Pos = [X,Y,Z],
    hsr_existing_objects(Objects),
    member(Instance, Objects),
    object_dimensions(Instance, Depth, Width, Height),
    object_tf_frame(Instance, ObjectFrame),
    tf_transform_point(map, ObjectFrame, Pos, [RelX, RelY, RelZ]),
    (Depth >= Width % ignore the orientation of the object
        -> Size = Depth
        ; Size = Width),
    min_space_between_objects(Threshold2),
    Threshold = Threshold1 + Threshold2,
    abs(RelX) < (Size / 2) + Threshold,
    abs(RelY) < (Size / 2) + Threshold,
    abs(RelZ) < Height. % assuming, the object we want to place has about the same height as the object already placed.

surface_pose_in_map(SurfaceLink, Pose) :-
    urdf_tf_frame(SurfaceLink, Frame),
    hsr_lookup_transform(map, Frame, [X,Y,Z], Rotation),
    Pose = [[X,Y,Z], Rotation].

%%%%%%%%%%%%%%%  Supportable by surface  %%%%%%%%%%%%%%%%%
    

position_supportable_by_ground(ZPos) :-
    number(ZPos),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= ZPos,
    ThBelow =< ZPos.

position_supportable_by_ground([_,_,Z]) :-
    position_supportable_by_ground(Z).

%TODO Support ground
is_legal_obj_position([X, Y, Z]) :-
    is_surface(Surface),
    position_supported_by_surface([X, Y, Z], Surface).
    %urdf_tf_frame(Surface, SurfaceFrame),
    %tf_transform_pose('map',SurfaceFrame,pose([X, Y, Z],[0,0,0,1]),pose(RelPosition,_)),   
    %position_supported_by_surface(RelPosition, Surface).


locations_not_visited(Locations) :-
    surfaces_not_visited(Surfaces),
    findall([Room, Furniture, Surface],
    (
        is_room(Room),
        member(Surface, Surfaces),
        has_surface(Furniture, Surface),
        in_room(Furniture, Room)
    ),
    Locations).

locations_not_visited(Room, Locations) :-
    is_room(Room),
    surfaces_not_visited(Surfaces),
    findall([Furniture, Surface], 
    (
        member(Surface, Surfaces),
        has_surface(Furniture, Surface),
        in_room(Furniture, Room)
    ), 
    Locations).


place_objects :-
    hsr_existing_objects(Objects),
    forall(member(Object, Objects), 
    (
        ignore(object_at_location(Object, _, _, _))
    )).

object_at_location(Object, Room, Furniture, Surface) :-
    is_suturo_object(Object),
    once(has_location(Object, _)),!, % just check if it already has a Location
    object_in_room(Object, Room),
    object_on_furniture(Object, Furniture),
    once(object_supported_by_surface(Object, Surface)),
    !.

object_at_location(Object, Room, Furniture, Surface) :-
    is_suturo_object(Object),
    tell(has_type(ObjectLocation, soma:'Location')),
    once(tell(has_location(Object, ObjectLocation))),!,
    object_in_room(Object, Room),
    once(object_supported_by_surface(Object, Surface)),
    once(object_on_furniture(Object, Furniture)),
    !.

forget_object_at_location(Object) :-
    (
        has_location(Object, ObjectLocation),
        forall(triple(ObjectLocation, knowrob:'isInsideOf', _), tripledb_forget(ObjectLocation, knowrob:'isInsideOf', _)),
        forall(triple(ObjectLocation, knowrob:'isOntopOf', _), tripledb_forget(ObjectLocation, knowrob:'isOntopOf', _)),
        forall(triple(ObjectLocation, soma:'isSupportedBy', _), tripledb_forget(ObjectLocation, soma:'isSupportedBy', _))
    );
    not has_location(Object, _).

object_at_predefined_location(Object, RoomType, FurnitureType) :-
    object_in_predefined_room(Object, RoomType),
    object_on_predefined_furniture(Object, FurnitureType).

objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object,
    (
        object_at_predefined_location(Object, RoomType, FurnitureType)
    ), 
    Objects).


misplaced_objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_at_predefined_location(Object, RoomType, FurnitureType),
        is_misplaced(Object)
    ), 
    Objects).

objects_at_same_predefined_location(Object, Objects) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    objects_at_predefined_location(Objects, RoomType, FurnitureType).


surface_at_predefined_location(Surface, RoomType, FurnitureType) :-
    has_surface(Furniture, Surface),
    has_type(Furniture, FurnitureType),
    subclass_of(FurnitureType, soma:'DesignedFurniture'),
    furniture_in_room(Furniture, Room),
    has_type(Room, RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').

surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType) :-
    findall(Surface,
    (
        surface_at_predefined_location(Surface, RoomType, FurnitureType)
    ),
    Surfaces).

robot_in_room(Room) :-
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, 'base_footprint', pose(RobotPosition, _)),
    position_in_room(RobotPosition, Room),
    !.

%robot_in_room(Room) :-
%    ask(has_type(Room, hsr_rooms:'Outside')),
%    !.

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

surface_in_room(Surface, Room) :-
    has_surface(Furniture, Surface),
    furniture_in_room(Furniture, Room).

object_in_predefined_room(Object, RoomType) :-
    has_predefined_location(Object, Location),
    triple(Location, knowrob:'isInsideOf', RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').

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

object_on_predefined_furniture(Object, FurnitureType) :-
    has_predefined_location(Object, Location),
    (triple(Location, knowrob:'isOntopOf', FurnitureType);
    triple(Location, knowrob:'isInsideOf', FurnitureType)),
    subclass_of(FurnitureType, soma:'DesignedFurniture').

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


position_in_room(Position, Room) :-
    is_room(Room),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(Position, CornerPoints),
    !.


position_in_room(Position, Room) :-
    has_type(Room, hsr_rooms:'Outside'),
    !.

pose_is_outside(Position) :-
    position_in_room(Position, Room),
    has_type(Room, hsr_rooms:'Outside').


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




furnitures_in_room(Room, Furnitures) :-
    all_furnitures(AllFurnitures),
    findall(Furniture, 
    (
        member(Furniture, AllFurnitures),
        furniture_in_room(Furniture, Room)
    ),
    Furnitures).


surfaces_in_room(Room, Surfaces) :-
    furnitures_in_room(Room, Furnitures),
    findall(Surface, 
    (
        member(Furniture, Furnitures),
        has_surface(Furniture, Surface)
    ),
    Surfaces).


objects_in_room(Room, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_in_room(Object, Room)
    ),
    Objects).


objects_on_furniture(Furniture, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_on_furniture(Object, Furniture)
    ),
    Objects).


objects_supported_by_surface(Surface, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_supported_by_surface(Object, Surface)
    ),
    Objects).


objects_supported_by_surfaces(Surfaces, Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        member(Surface, Surfaces),
        object_supported_by_surface(Object, Surface)
    ),
    Objects).


has_predefined_location(Object, Location) :-
    once((
        has_type(Object, ObjectType),
        transitive(subclass_of(ObjectType, SupportedType)),
        triple(SupportedType, hsr_rooms:'hasPredefinedLocation', Location)
    )).


is_misplaced(Object) :-
    object_at_location(Object, Room, Furniture, _),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    not (has_type(Room, RoomType), has_type(Furniture, FurnitureType)).



object_pose(ObjID, ['map',Frame, Point, Rotation]) :-
    object_tf_frame(ObjID,Frame),
    hsr_lookup_transform(map, Frame, Point, Rotation).


distance_to_robot(Thing, Distance) :-
    ( is_surface(Thing)
    -> urdf_tf_frame(Thing, Frame)
    ; object_tf_frame(Thing,Frame)
    ),  
    hsr_lookup_transform(map, Frame, [OX, OY, OZ], _),
    hsr_lookup_transform(map,'base_footprint',[BX,BY,BZ],_),
    DX is (OX - BX),
    DY is (OY - BY),
    DZ is (OZ - BZ),
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.





