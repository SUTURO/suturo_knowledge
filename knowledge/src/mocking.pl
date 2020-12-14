:- module(mocking,
    [
      create_object_on_surface/1,
      setup/0,
      advanced_setup/0,
      mock_new_object_place/2
    ]).

:- rdf_meta
    create_object_on_surface(?).


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
        _, 
        [0.05, 0.075, 0.2], 
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

% This simulates the behavior of an Object, that gets taken by the 
% Gripper and than released at a different place.
% i.e. it simulates the result of
% attach_object_to_gripper(Object),
% release_object_from_gripper([Translation, Rotation]).
mock_new_object_place(Object, [Translation, Rotation]) :-
    position_supportable_by_surface(Translation, _),
    hsr_belief_at_update(Object, [map, _, Translation, Rotation]),
    place_object(Object),
    group_target_objects,
    !.

mock_new_object_place(_, [Translation, _]) :-
    not(position_supportable_by_surface(Translation, _)),
    writeln("The Position is not above any surface"),
    !.

mock_new_object_place(_, _) :-
    writeln("The Position is o.k., but place_object/1 or group_target_objects/0 returned false.").
