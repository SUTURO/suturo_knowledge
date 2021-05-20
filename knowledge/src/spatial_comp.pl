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
        object_in_room/2,
        surface_in_room/2,
        furniture_in_room/2,
        robot_in_room/1,
        object_at_location/4,
        object_on_furniture/2,
        furnitures_in_room/2,
        surfaces_in_room/2,
        objects_in_room/2,
        locations_not_visited/1,
        locations_not_visited/2,
        predefined_location/3,
        object_in_room/2,
        is_misplaced/2,
        surfaces_according_to_predefined_location/3
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
    surface_tf_frame(SurfaceLink, Frame),
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


is_legal_obj_position([X, Y, Z]) :-
    position_supported_by_surface([X, Y, Z], _).


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


object_at_location(Object, Room, Furniture, Surface) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    in_room(Object, Room),
    on_furniture(Object, Furniture),
    supported_by_surface(Object, Surface),
    !.

object_at_location(Object, Room, Furniture, Surface) :-
    is_suturo_object(Object),
    tell(has_type(ObjectLocation, soma:'Location')),
    tell(has_location(Object, ObjectLocation)),
    in_room(Object, Room),
    on_furniture(Object, Furniture),
    supported_by_surface(Object, Surface),
    !.

object_at_predefined_location(Object, RoomType, FurnitureType) :-
    is_suturo_object(object),
    in_predefined_room(Object, RoomType),
    on_predefined_furniture(Object, FurnitureType).

robot_in_room(Room) :-
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, 'base_footprint', pose(RobotPosition, _)),
    has_type(Room, hsr_rooms:'Room'),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(RobotPosition, CornerPoints).

object_in_room(Object, Room) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    triple(ObjectLocation, knowrob:'isInsideOf', Room),
    is_room(Room).

object_in_room(Object, Room) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    object_tf_frame(Object, ObjectFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, ObjectFrame, pose(ObjectPosition, _)),
    has_type(Room, hsr_rooms:'Room'),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(ObjectPosition, CornerPoints),
    forall(
    (
        triple(ObjectLocation, knowrob:'isInsideOf', CurrentRoom), 
        has_type(CurrentRoom, hsr_rooms:'Room')
    ), 
    update(triple(ObjectLocation, knowrob:'isInsideOf', Room))).


object_in_room(Object, Room) :-
    is_suturo_object(Object),
    object_tf_frame(Object, ObjectFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, ObjectFrame, pose(ObjectPosition, _)),
    has_type(Room, hsr_rooms:'Room'),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(ObjectPosition, CornerPoints),
    tell(has_type(ObjectLocation, soma:'Location')),
    tell(has_location(Object, ObjectLocation)),
    tell(triple(ObjectLocation, knowrob:'isInsideOf', Room)).


furniture_in_room(Furniture, Room) :-
    is_furniture(Furniture),
    once(has_location(Furniture, FurnitureLocation)),
    triple(FurnitureLocation, knowrob:'isInsideOf', Room),
    !.

furniture_in_room(Furniture, Room) :-
    is_furniture(Furniture),
    once(has_location(Furniture, FurnitureLocation)),
    urdf_tf_frame(Furniture, FurnitureFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, FurnitureFrame, pose(ObjectPosition, _)),
    has_type(Room, hsr_rooms:'Room'),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(ObjectPosition, CornerPoints),
    update(triple(FurnitureLocation, knowrob:'isInsideOf', Room)),
    !.

furniture_in_room(Furniture, Room) :-
    is_furniture(Furniture),
    urdf_tf_frame(Furniture, FurnitureFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(Origin, FurnitureFrame, pose(ObjectPosition, _)),
    has_type(Room, hsr_rooms:'Room'),
    room_corner_point_positions(Room, CornerPoints),
    point_in_polygon(ObjectPosition, CornerPoints),
    tell(has_type(FurnitureLocation, soma:'Location')),
    tell(has_location(Furniture, FurnitureLocation)),
    update(triple(FurnitureLocation, knowrob:'isInsideOf', Room)),
    !.

surface_in_room(Surface, Room) :-
    is_surface(Surface),
    has_surface(Furniture, Surface),
    furniture_in_room(Furniture, Room).

object_in_predefined_room(Object, RoomType) :-
    has_predefined_location(Object, Location),
    triple(Location, knowrob:'isInsideOf', RoomType).

object_on_furniture(Object, Furniture) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    triple(ObjectLocation, knowrob:'isInsideOf', Furniture),
    is_furniture(Furniture).

object_on_furniture(Object, Furniture) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    triple(ObjectLocation, knowrob:'isOnTopOf', Furniture),
    is_furniture(Furniture).

object_on_furniture(Object, Furniture) :-
    is_suturo_object(Object),
    is_furniture(Furniture),
    has_surface(Furniture, Surface),
    supported_by_surface(Object, Surface).

object_on_predefined_furniture(Object, FurnitureType) :-
    has_predefined_location(Object, Location),
    (triple(Location, knowrob:'isOntopOf', SurfaceType);
    triple(Location, knowrob:'isInsideOf', SurfaceType)).

object_supported_by_surface(Object, Surface) :-
    is_suturo_object(Object),
    has_location(Object, ObjectLocation),
    triple(ObjectLocation, soma:'isSupportedBy', Surface).

object_supported_by_surface(Object, Surface) :-
    is_suturo_object(Object),
    is_surface(Surface),
    object_tf_frame(Object, ObjectFrame),
    urdf_tf_frame(Surface, SurfaceFrame),
    tf_lookup_transform(map, ObjectFrame, pose(Position, _)),
    position_supported_by_surface(Position, Surface),
    has_location(Object, ObjectLocation),
    tell(triple(ObjectLocation, soma:'isSupportedBy', Surface)).

position_supported_by_surface(Position, Surface) :-
    is_surface(Surface),
    surface_dimensions(Surface, Depth, Width, _),
    threshold_surface(ThAbove, ThBelow),
    urdf_tf_frame(Surface, SurfaceFrame),
    writeln(SurfaceFrame),
    tf_transform_point(map, SurfaceFrame, Position, [X, Y, Z]),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    Depth/2 >= abs(X).


%position_supportable_by_surface(Position, ground) :-
%    position_supportable_by_ground(Position).



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

has_predefined_location(Object, Location) :-
    once(
    (
        has_type(Object, ObjectType),
        transitive(subclass_of(ObjectType, SupportedType)),
        triple(SupportedType, hsr_rooms:'hasPredefinedLocation', Location) 
    )).

is_misplaced(Object) :-
    object_at_location(Object, Room, Furniture, _),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    has_type(Room, RoomType),
    has_type(Furniture, FurnitureType).



object_pose(ObjID, ['map',Frame, Point, Rotation]) :-
    object_tf_frame(ObjID,Frame),
    hsr_lookup_transform(map, Frame, Point, Rotation).


distance_to_robot(Thing, Distance) :-
    ( is_surface(Thing)
    -> surface_tf_frame(Thing, Frame)
    ; object_tf_frame(Thing,Frame)
    ),  
    hsr_lookup_transform(map, Frame, [OX, OY, OZ], _),
    hsr_lookup_transform(map,'base_footprint',[BX,BY,BZ],_),
    DX is (OX - BX),
    DY is (OY - BY),
    DZ is (OZ - BZ),
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


predefined_location(Object, RoomType, SurfaceType) :-
    once((
        has_type(Object, ObjectClass),
        transitive(subclass_of(ObjectClass, SupportedClass)),
        ( in_room(SupportedClass, _), has_location(SupportedClass,_)
        -> 
        (        
            in_room(SupportedClass, RoomType),
            has_location(SupportedClass, SurfaceType)
        )
        ;   
        (
            SurfaceType = 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Bucket',
            object_in_room(Object, Room),
            triple(Room, hsr_rooms:'hasRoomTypeRole', RoomTypeInstance),
            has_type(RoomTypeInstance, RoomType)
        ))
    )).


is_misplaced(Object, State) :-
    predefined_location(Object, PredefRoomType, PredefSurfaceType),
    object_in_room(Object, Room),
    triple(Room, hsr_rooms:'hasRoomTypeRole', CurrentRoomType),
    object_supportable_by_surface(Object, Surface),
    triple(Surface, hsr_objects:'isSurfaceType', CurrentSurfaceType),
    has_type(CurrentRoomType, RoomType), 
    has_type(CurrentSurfaceType, SurfaceType),
    ( same_as(PredefRoomType, RoomType), same_as(PredefSurfaceType, SurfaceType)
    -> State = 0
    ;
    State = 1
    ).


surfaces_according_to_predefined_location(RoomType, SurfaceType, MatchingSurfaces) :-
    all_surfaces_of_type(SurfaceType, Surfaces),
    all_rooms_of_type(RoomType, Rooms),
    findall(Surface, 
    (
        member(Surface, Surfaces),
        member(Room, Rooms),
        object_in_room(Surface, Room)
    ),
    MatchingSurfaces).


