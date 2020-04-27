:- module(mocking,
    [
      create_object_on_surface/1,
      setup/0,
      advanced_setup/0
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).

:- rdf_meta
    create_object_on_surface(?).


create_objects_on_table :-
    create_banana_on_table,
    create_milk_on_table,
    create_coffee_on_table,
    create_unknown_on_table.
    %group_table_objects.

create_object_on_surface(Surface) :-
    rdf_urdf_link_collision(Surface, box(Depth, Width, _), _),
    YMin is (-1) * (Width/2) + 0.1,
    YMax is (Width/2) - 0.1,
    XMin is Depth - 0.1,
    XMax is + 0.1,
    surface_front_edge_center_frame(Surface, Frame),
    find_random_suitable_pos_(XMin, XMax, YMin, YMax, Frame, [RelativeX,RelativeY]),
    tf_transform_point(Frame, map, [RelativeX, RelativeY, 0], Pos),
    Transform = ['map', _, Pos, [0,0,1,0]],
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Banana',
        0.8, 
        Transform,
        0.05, 
        _, 
        [0.2, 0.075, 0.2], 
        box,
        0.8,
        [0.0,0.0,0.0,0.0],
        0.9),
    !.

find_random_suitable_pos_(XMin, XMax, YMin, YMax, Frame, Pos) :-
    random(XMin, XMax, X),
    random(YMin, YMax, Y),
    Transform = [Frame, _, [X,Y,0], [0,0,1,0]],
    (   hsr_existing_object_at(Transform, 0.01, _)
        -> find_random_suitable_pos_(XMin, XMax, YMin, YMax, Pos)
        ; Pos = [X,Y]
    ).

setup :-
    make_all_tables_source,
    make_all_shelves_target.

advanced_setup :-
    table_surfaces(A),
    member(B,A), !,
    create_object_on_surface(B),
    create_object_on_surface(B),
    create_object_on_surface(B),
    create_object_on_surface(B),
    setup.
