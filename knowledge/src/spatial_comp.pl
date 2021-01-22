:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3, % deprecated
        hsr_existing_object_at/2,
        hsr_existing_object_at_thr/2,
        hsr_existing_object_at_thr/3,
        surface_pose_in_map/2,
        object_supportable_by_surface/2,
        position_supportable_by_surface/2,
        distance_to_robot/2,
        %Debug
        relative_position_supportable_by_surface/2
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

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
    object_pose(Instance, OldPose),
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
    object_frame_name(Object, ObjectFrame),
    hsr_lookup_transform(SurfaceFrame, ObjectFrame, Position, _),
    relative_position_supportable_by_surface(Position, Surface).
    
object_supportable_by_surface(Object, ground):-
    object_frame_name(Object, Frame),
    hsr_lookup_transform(map, Frame, [_,_,Z], _),
    position_supportable_by_ground(Z).

position_supportable_by_surface(Position, Surface) :-
    all_surfaces(Surfaces),
    member(Surface, Surfaces),
    surface_front_edge_center_frame(Surface, Frame),
    tf_transform_point(map, Frame, Position, RelativePosition),
    relative_position_supportable_by_surface(RelativePosition, Surface),
    !.
    
position_supportable_by_surface(Position, ground) :-
    position_supportable_by_ground(Position).

relative_position_supportable_by_surface([X,Y,Z],Surface) :-
    is_table(Surface),
    surface_dimensions(Surface, Depth, Width, _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    0 < X,
    Depth >= X.

relative_position_supportable_by_surface([X,Y,Z], Surface) :-
    ( is_shelf(Surface) ; is_bucket(Surface) ),
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


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
