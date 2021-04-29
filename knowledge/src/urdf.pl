:- module(urdf,
    [
        load_surfaces_from_param/1,
        get_surface_id_by_name/2,
        surface_tf_frame/2,
        surface_frame_add_prefix_/2,
        surface_front_edge_center_frame/2,
        surface_front_edge_center_pose/2,
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
    urdf_set_pose_to_origin(URDF,map),
    urdf_link_names(URDF,Links),
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

surface_front_edge_center_pose(Surface,[Trans,Rot]):-
    surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame),
    tf_lookup_transform('map', FrontEdgeCenterFrame, pose(Trans,Rot)).


surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Shelf
    is_shelf(Surface),
    FrontEdgeCenterFrame = Surface.
    %surface_frame_with_prefix_(Surface, FrontEdgeCenterFrame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Table or a Bucket
    (is_table(Surface); is_bucket(Surface)),
    atom_length(Surface,L),
    C is L - 7,
    sub_atom(Surface, 0, C, _, Name), % cuts away the Postfix "_center" (the last 7 letters)
    atom_concat(Name, '_front_edge_center', FrontEdgeCenterFrame).

    
surface_frame_add_prefix_(SurfaceName, Surface_with_Prefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, SurfaceName, Surface_with_Prefix).

surface_dimensions(Surface, Depth, Width, Height) :- % adapted to new knowrob
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF,Surface, box(Depth, Width, Height),_).

