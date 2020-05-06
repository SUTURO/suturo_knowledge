:- module(urdf,
    [
        load_surfaces_from_param/1,
        get_surface_id_by_name/2,
        urdf_frame/2,
        object_frame/2,
        surface_frame_/2,
        surface_front_edge_center_frame/2,
        surface_dimensions/4
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- rdf_meta
    load_surfaces_from_param(-).
    

/**
* is called as inital_goal in the launch file
*/
load_surfaces_from_param(Param):-
    (once(rdfs_individual_of(_, urdf:'Robot'))
    -> write("Surfaces are allready loaded. Restart the knowledgebase to load a diffrent URDF")
    ;  kb_create(urdf:'Robot', RobotNew),rdf_urdf_load_param(RobotNew, Param),
        forall(supporting_surface(SurfaceLink),
        assert_surface_types(SurfaceLink))
    ).


%% takes names like table_1_center or shelf_floor_4_piece or ground.
%% Returns false if name is not a registered surface.
get_surface_id_by_name(Name, SurfaceId):-
    (rdf_urdf_name(SurfaceId, Name), all_surfaces(Surfaces), member(SurfaceId, Surfaces)
        -> true
        ;  (Name = ground
            -> SurfaceId = ground
            ; false
        )
    ).

urdf_frame(Thing, Frame):-
    is_object(Thing),
    object_frame_name(Thing, Frame).

urdf_frame(Thing, Frame):-
    is_surface(Thing),
    surface_front_edge_center_frame(Thing, Frame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Shelf
    is_shelf(Surface),
    surface_frame(Surface, FrontEdgeCenterFrame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :- % in case it's a Table or a Bucket
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
    
surface_frame_(Surface, Frame) :-
    rdf_urdf_name(Surface, Name),
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, Name, Frame).

surface_dimensions(Surface, Width, Depth, Height) :-
    rdf_urdf_link_collision(Surface, box(Width, Depth, Height), _).
