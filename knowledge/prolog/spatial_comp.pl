:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3,
        surface_pose_in_map/2,
        object_supportable_by_surface/2,
        position_supportable_by_surface/2,
        distance_to_robot/2,
        % Type cast
        urdf_frame/2,
        object_frame/2,
        surface_frame/2,
        surface_front_edge_center_frame/2,
        %Debug
        relative_position_supportable_by_surface/2
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).

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
    rdfs_individual_of(Instance, dul:'PhysicalObject'),
    rdf_has(Instance, hsr_objects:'supportable', true),
    object_pose(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).


surface_pose_in_map(SurfaceLink, Pose) :-
    urdf_frame(SurfaceLink, Frame),
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
    relative_position_supportable_by_surface(RelativePosition, Surface).
    
position_supportable_by_surface(Position, ground) :-
    position_supportable_by_ground(Position).

relative_position_supportable_by_surface([X,Y,Z],Surface) :-
    is_table(Surface),
    rdf_urdf_link_collision(Surface, box(Depth, Width, _), _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(Y),
    0 < X,
    Depth >= X.

relative_position_supportable_by_surface([X,Y,Z], Surface) :-
    ( is_shelf(Surface) ; is_bucket(Surface) ),
    rdf_urdf_link_collision(Surface, box(Depth, Width, _), _),
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
    urdf_frame(Thing, Frame),
    hsr_lookup_transform(map, Frame, [OX, OY, OZ], _),
    hsr_lookup_transform(map,'base_footprint',[BX,BY,BZ],_),
    DX is (OX - BX),
    DY is (OY - BY),
    DZ is (OZ - BZ),
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.

urdf_frame(Thing, Frame):-
    is_object(Thing),
    object_frame_name(Thing, Frame).

urdf_frame(Thing, Frame):-
    is_surface(Thing),
    surface_front_edge_center_frame(Thing, Frame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Shelf
    is_shelf(Surface),
    surface_frame(Surface, FrontEdgeCenterFrame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a table
    rdf_urdf_name(Surface, FullName),
    sub_atom(FullName, 0, _, 7, Name), % cuts away the Suffix "_center" (the last 7 letters)
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, Name, Part1),
    surface_suffix(Surface, Suffix),
    atom_concat(Part1, Suffix, FrontEdgeCenterFrame).

surface_suffix(Surface, Suffix) :-
    is_table(Surface),
    Suffix = "_front_edge_center".

surface_suffix(Surface, Suffix) :-
    is_bucket(Surface),
    Suffix = "_surface_center".

% deprecated.
% Should be deleted if not used.
object_frame(Object, Frame) :-
    object_frame_name(Object, Frame).
    
surface_frame(Surface, Frame) :-
    rdf_urdf_name(Surface, Name),
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, Name, Frame).
