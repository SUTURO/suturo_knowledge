:- module(urdf,
    [
        load_surfaces_from_param/1,
        get_surface_id_by_name/2,
        surface_frame_add_prefix_/2,
        urdf_frame_add_prefix_/2,
        surface_center_pose/2,
        surface_front_edge_center_pose/2,
        surface_dimensions/4,
        object_tf_frame/2,
        urdf_tf_frame/2,
        has_urdf_name/2
    ]).


%:- rdf_meta().
    

/**
* is called as inital_goal in the launch file
*/
load_surfaces_from_param(Param):-
    ros_param_get_string(Param,S), % S is the urdf file (a xml file) as a string
    get_urdf_id(URDF),
    urdf_load_xml(URDF,S),
    get_urdf_origin(Origin),
    urdf_set_pose_to_origin(URDF,Origin),
    urdf_link_names(URDF,Links)
    %init_surface_types,
%    forall(
%    ( member(Link, Links)),
%    ((supporting_surface(Link) % if supporting Surface
%        -> assert_surface_types(Link)) % assert it as a type
%        ; true   % else do nothing
%    ))
    .


%% takes names like table_1_center or shelf_floor_4_piece or ground.
%% Returns false if name is not a registered surface.
get_surface_id_by_name(Name, SurfaceId):-
    (all_surfaces(Surfaces), member(SurfaceId, Surfaces)
        -> true
        ;  (Name = ground
            -> SurfaceId = ground
            ; false
        )
    ).


object_tf_frame(Object, Frame) :-
    ( sub_string(Object,_,_,_,"#")
    -> split_string(Object, "#", "", [_, Frame])
    ;
    Frame = Object
    ).

urdf_tf_frame(UrdfInstance, Frame) :-
    has_urdf_name(UrdfInstance, UrdfName),
    urdf_frame_add_prefix_(UrdfName, Frame).


surface_center_pose(Surface, [Position, Rotation]):-
    has_urdf_name(Surface, CenterTFFrame),
    tf_lookup_transform('map', CenterTFFrame, pose(Position,Rotation)).

surface_front_edge_center_pose(Surface, [Position, Rotation]):-
    has_urdf_name(Surface, SurfaceLink),
    surface_dimensions(Surface,X,_,_),
    NegHalfX is X / -2,
    tf_transform_point(SurfaceLink, map, [NegHalfX, 0, 0], Position),
    tf_lookup_transform('map', SurfaceLink, pose(_,Rotation)).


    
surface_frame_add_prefix_(SurfaceName, Surface_with_Prefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, SurfaceName, Surface_with_Prefix).

urdf_frame_add_prefix_(UrdfName, UrdfNameWithPrefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, UrdfName, UrdfNameWithPrefix).

surface_dimensions(Surface, Depth, Width, Height) :- % adapted to new knowrob
    get_urdf_id(URDF),
    has_urdf_name(Surface, SurfaceLink),
    urdf_link_collision_shape(URDF,SurfaceLink, box(Depth, Width, Height),_).

has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).


    

