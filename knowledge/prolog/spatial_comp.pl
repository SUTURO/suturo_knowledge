:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3,
        surface_pose_in_map/2,
        object_supportable_by_surface/2,
        position_supportable_by_surface/2,
        distance_to_robot/2
    ]).

:- rdf_meta
    hsr_lookup_transform(r,r,?,?),
    hsr_existing_object_at(r,r,?),
    joint_abs_position(r,?),
    quaternion_to_euler(r,?),
    euler_to_quaternion(r,?).



hsr_lookup_transform(SourceFrame, TargetFrame, Translation, Rotation) :-
    tf_lookup_transform(SourceFrame, TargetFrame, pose(Translation,Rotation)).

hsr_existing_object_at(Pose, Threshold, Instance) :-
    rdf(Instance, rdf:type, owl:'NamedIndividual', belief_state),
    rdfs_individual_of(Instance, hsr_objects:'Item'),
    object_pose(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).


surface_pose_in_map(SurfaceLink, Pose) :-
    urdf_frame(SurfaceLink, Frame),
    hsr_lookup_transform(map, Frame, Translation, Rotation),
    Pose = [Translation, Rotation].

%%%%%%%%%%%%%%%  Supportable by surface  %%%%%%%%%%%%%%%%%5

% finds and returns surfaces the object might be standing on.
object_supportable_by_surface(Object, Surface):-
    all_surfaces(Surfaces),
    member(Surface,Surfaces),
    surface_front_edge_center_frame(Surface, SurfaceFrame),
    object_frame(Object, ObjectFrame),
    hsr_lookup_transform(SurfaceFrame, ObjectFrame, Position, _),
    relative_position_supportable_by_surface(Position, Surface).
    
object_supportable_by_surface(Object, ground):-
    object_frame(Object, Frame),
    hsr_lookup_transform(map, Frame, [_,_,Z], _),
    position_supportable_by_ground(Z).

position_supportable_by_surface(Position, Surface) :-
    all_surfaces(Surfaces),
    member(Surface, Surfaces),
    surface_front_edge_center_frame(Surface, Frame),
    tf_transform_point(map, Frame, Position, RelativePosition),
    relative_position_supportable_by_surface(RelativePosition, Surface).
    
position_supportable_by_surface(Position, ground) :-
    position_supportable_by_ground(Position).

relative_position_supportable_by_surface([X,Y,Z],Surface) :-
    rdf_urdf_link_collision(Surface, box(Width, Depth, _), _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(X),
    0 >= Y,
    Depth*(-1) =< Y.

position_supportable_by_ground(ZPos) :-
    number(ZPos),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= ZPos,
    ThBelow =< ZPos.

position_supportable_by_ground([_,_,Z]) :-
    position_supportable_by_ground(Z).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


distance_to_robot(Thing, Distance) :-
    urdf_frame(Thing, Frame),
    hsr_lookup_transform(map, Frame, [OX, OY, OZ], _),
    hsr_lookup_transform(map,'base_footprint',[BX,BY,BZ],_),
    DX is (OX - BX),
    DY is (OY - BY),
    DZ is (OZ - BZ),
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.
