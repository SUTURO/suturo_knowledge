:- module(urdf,
    [
        load_surfaces_from_param/1,
        get_surface_id_by_name/2,
        surface_tf_frame/2,
        surface_frame_add_prefix_/2,
        surface_front_edge_center_frame/2,
        surface_dimensions/4,
        object_tf_frame/2
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
    urdf_link_names(URDF,Links),
    init_surface_types,
    forall(
    ( member(Link, Links)),
    ((supporting_surface(Link) % if supporting Surface
        -> assert_surface_types(Link)) % assert it as a type
        ; true   % else do nothing
    ))
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


%% gives the tf frame given the SurfaceID
surface_tf_frame(Surface, Frame):-
    is_surface(Surface),
    surface_front_edge_center_frame(Surface, Frame).

object_tf_frame(Object, Frame) :-
    ( sub_string(Object,_,_,_,"#")
    -> split_string(Object, "#", "", [_, Frame])
    ;
    Frame = Object
    ).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Shelf
    is_shelf(Surface),
    FrontEdgeCenterFrame = Surface.
    %surface_frame_with_prefix_(Surface, FrontEdgeCenterFrame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it is a Table or a Bucket
    sub_atom(Surface, 0, _, 7, Name), % cuts away the Suffix "_center" (the last 7 letters)
    %urdf_surface_prefix(Prefix), % /kitchen_desciption
    %atom_concat(Prefix, Name, Part1), % results in /kitchen_desciption/table_1
    surface_suffix(Surface, Suffix), % front_edge_center for tables / surface_center for bucket
    atom_concat(Name, Suffix, FrontEdgeCenterFrame). % /kitchen_desciption/table_1_front_edge_center

surface_suffix(Surface, Suffix) :-
    is_table(Surface),
    Suffix = "_front_edge_center".

surface_suffix(Surface, Suffix) :-
    is_bucket(Surface),
    Suffix = "_surface_center".
    
surface_frame_add_prefix_(SurfaceName, Surface_with_Prefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, SurfaceName, Surface_with_Prefix).

surface_dimensions(Surface, Depth, Width, Height) :- % adapted to new knowrob
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF,Surface, box(Depth, Width, Height),_).

object_frame_name(Object, FrameName) :-
    ( sub_string(Object,_,_,_,"#")
    -> split_string(Object, "#", "", [_, FrameName])
    ;
    FrameName = Object
    ).

has_urdf_name(Object, URDFName) :-
    triple(Object, urdf:'hasURDFName', URDFName).
    

