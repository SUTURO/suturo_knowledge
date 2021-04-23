:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3, % deprecated
        hsr_existing_object_at/2,
        hsr_existing_object_at_thr/2,
        hsr_existing_object_at_thr/3,
	object_pose/2,
        surface_pose_in_map/2,
        object_supportable_by_surface/2,
        position_supportable_by_surface/2,
        distance_to_robot/2,
        %Debug
        relative_position_supportable_by_surface/2,
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

%%%%%%%%%%%%%%%  Supportable by surface  %%%%%%%%%%%%%%%%%5

% finds and returns surfaces the object might be standing on.
object_supportable_by_surface(Object, Surface):-
    all_surfaces(Surfaces),
    member(Surface,Surfaces),
    surface_front_edge_center_frame(Surface, SurfaceFrame),
    object_tf_frame(Object, ObjectFrame),
    hsr_lookup_transform(SurfaceFrame, ObjectFrame, Position, _),
    relative_position_supportable_by_surface(Position, Surface).
    
object_supportable_by_surface(Object, ground):-
    object_tf_frame(Object, Frame),
    hsr_lookup_transform(map, Frame, [_,_,Z], _),
    position_supportable_by_ground(Z).

position_supportable_by_surface(Position, Surface) :-
    all_surfaces(Surfaces),
    member(Surface, Surfaces),
    surface_front_edge_center_frame(Surface, Frame),
    tf_transform_point(map, Frame, Position, RelativePosition), % rospolog
    relative_position_supportable_by_surface(RelativePosition, Surface),
    !.
    
position_supportable_by_surface(Position, ground) :-
    position_supportable_by_ground(Position).

relative_position_supportable_by_surface([X,Y,Z], Surface) :-
    ( is_table(Surface); is_dishwasher(Surface); is_bed(Surface); is_sideboard(Surface); is_sink(Surface) ),
    surface_dimensions(Surface, Depth, Width, _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    0 < X,
    Depth >= X.

relative_position_supportable_by_surface([X,Y,Z], Surface) :-
    ( is_shelf(Surface); is_cabinet(Surface) ; is_bucket(Surface) ),
    surface_dimensions(Surface, Depth, Width, _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    Depth/2 >= abs(X).

position_supportable_by_ground(ZPos) :-
    number(ZPos),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= ZPos,
    ThBelow =< ZPos.

position_supportable_by_ground([_,_,Z]) :-
    position_supportable_by_ground(Z).


%%%%%%%%%%%%%% in room %%%%%%%%%%%%%%

object_in_room(Object, Room) :-
    object_frame_name(Object, ObjFrame),
    get_urdf_origin(Origin),
    tf_lookup_transform(ObjFrame, Origin, pose([XObj, YObj, _], _)),
    has_type(Room, hsr_rooms:'Room'),
    object_frame_name(Room, RoomFrame),
    tf_lookup_transform(RoomFrame, Origin, pose([XRoom, YRoom, _], _)),
    object_dimensions(Room, Width, Depth, _),
    XObj > XRoom - Width/2,
    XObj < XRoom + Width/2,
    YObj > YRoom - Depth/2,
    YObj < YRoom + Depth/2.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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


