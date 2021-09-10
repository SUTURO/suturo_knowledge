:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3,
        hsr_existing_object_at_thr/2,
        object_pose/2,
        surface_pose_in_map/2,
        compareDistances/3,
        surface_dimensions/4,
        surface_front_edge_center_pose/2,
        surface_center_pose/2
    ]).

:- use_module(library('model/environment/surfaces'),
    [
        is_surface/1
    ]).
:- use_module(library('model/objects/object_info'), [hsr_existing_objects/1]).

hsr_lookup_transform(SourceFrame, TargetFrame, Translation, Rotation) :-
    tf_lookup_transform(SourceFrame, TargetFrame, pose(Translation,Rotation)).

hsr_existing_object_at(Pose, Threshold, Instance) :-
    has_type(Instance, owl:'NamedIndividual'),
    instance_of(Instance, dul:'PhysicalObject'),
    triple(Instance, hsr_objects:'supportable', true),
    is_at(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).

hsr_existing_object_at([X,Y,Z], Instance) :-
    hsr_existing_object_at_thr([X,Y,Z], 0, Instance).

hsr_existing_object_at([map,_,Pos, _], Instance) :-
    hsr_existing_object_at(Pos, Instance).

hsr_existing_object_at_thr([X,Y,Z], Threshold) :-
    hsr_existing_object_at_thr([X,Y,Z], Threshold, _).


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
    abs(RelZ) < Height.

surface_pose_in_map(SurfaceLink, Pose) :-
    urdf_tf_frame(SurfaceLink, Frame),
    hsr_lookup_transform(map, Frame, [X,Y,Z], Rotation),
    Pose = [[X,Y,Z], Rotation].


surface_center_pose(Surface, [Position, Rotation]):-
    has_urdf_name(Surface, CenterTFFrame),
    tf_lookup_transform('map', CenterTFFrame, pose(Position,Rotation)).

surface_front_edge_center_pose(Surface, [Position, Rotation]):-
    has_urdf_name(Surface, SurfaceLink),
    surface_dimensions(Surface,X,_,_),
    NegHalfX is X / -2,
    tf_transform_point(SurfaceLink, map, [NegHalfX, 0, 0], Position),
    tf_lookup_transform('map', SurfaceLink, pose(_,Rotation)).


surface_dimensions(Surface, Depth, Width, Height) :-
    get_urdf_id(URDF),
    has_urdf_name(Surface, SurfaceLink),
    urdf_link_collision_shape(URDF,SurfaceLink, box(Depth, Width, Height),_).


object_pose(ObjID, ['map',Frame, Point, Rotation]) :-
    object_tf_frame(ObjID,Frame),
    hsr_lookup_transform(map, Frame, Point, Rotation).


compareDistances(Order, Thing1, Thing2) :-
    distance_to_robot(Thing1, Dist1),
    distance_to_robot(Thing2, Dist2),
    (Dist1 = Dist2 % prevent predsort from deleting duplicate distances
        -> compare(Order, 0, Dist2)
        ; compare(Order, Dist1, Dist2))
    .


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

euclidean_distance([X1, Y1, Z1], [X2, Y2, Z2], Distance) :-
    XDiff is X1 - X2, YDiff is Y1 - Y2, ZDiff is Z1 - Z2,
    XSquare is XDiff * XDiff,
    YSquare is YDiff * YDiff,
    ZSquare is ZDiff * ZDiff,
    Sum is XSquare + YSquare + ZSquare,
    Distance is sqrt(Sum).


angle_to_quaternion(Angle, Quaternion) :-
    C is cos(Angle/2),
    S is sin(Angle/2),
    Quaternion = [0.0, 0.0, S, C].





