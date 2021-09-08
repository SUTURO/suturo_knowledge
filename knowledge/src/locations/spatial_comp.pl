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
        position_in_room/2
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





