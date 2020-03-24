:- module(mocking,
    [
      create_objects_on_table/0
    ]).


:- rdf_meta
    create_objects_on_table.


create_objects_on_table :-
    create_banana_on_table,
    create_icetee_on_table,
    create_coffee_on_table,
    create_unknown_on_table.


create_banana_on_table :-
    select_surface([2.025, 0.85, 0.47],_),
    create_object_at(hsr_objects:'Banana',
        ['map', _, [2.025, 0.85, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.0551572740078, 0.113258674741, 0.134232342243], 
        [0.0, 0.0, 0.0, 1.0]),
    place_object(ObjectInstance).

create_icetee_on_table :-
    select_surface([2.17, 0.9, 0.47],_),
    create_object_at(hsr_objects:'Banana',
        ['map', _, [2.17, 0.9, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.0551572740078, 0.113258674741, 0.134232342243], 
        [0.0, 0.0, 0.0, 1.0]),
    place_object(ObjectInstance).

create_coffee_on_table :-
    select_surface([2.275, 1.1, 0.47],_),
    create_object_at(hsr_objects:'Banana',
        ['map', _, [2.275, 1.1, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.0551572740078, 0.113258674741, 0.134232342243], 
        [0.0, 0.0, 0.0, 1.0]),
    place_object(ObjectInstance).

create_unknown_on_table :-
    select_surface([2.275, 0.65, 0.47],_),
    create_object_at(hsr_objects:'Banana',
        ['map', _, [2.275, 0.65, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.0551572740078, 0.113258674741, 0.134232342243], 
        [0.0, 0.0, 0.0, 1.0]),
    place_object(ObjectInstance).