:- module(mocking,
    [
      create_objects_on_table/0
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).

:- rdf_meta
    create_objects_on_table.


create_objects_on_table :-
    create_banana_on_table,
    create_milk_on_table,
    create_coffee_on_table,
    create_unknown_on_table.
    %group_table_objects.


create_banana_on_table_old :-
    select_surface([2.025, 0.85, 0.47],_),
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Banana',
        ['map', _, [2.025, 0.85, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.2, 0.075, 0.075], 
        [0.153, 0.17, 0.187, 1.0]),
    place_object(ObjectInstance).

create_banana_on_table :-
    select_surface([2.025, 0.85, 0.47],_),
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Banana',
        0.8,
        ['map', _, [2.025, 0.85, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.2, 0.075, 0.075], 
        box,
        0.4,
        [0.153, 0.17, 0.187, 1.0],
        0.9),
    place_object(ObjectInstance).

create_milk_on_table :-
    select_surface([2.17, 0.9, 0.47],_),
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Vollmilch',
        0.7,
        ['map', _, [2.17, 0.9, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.075, 0.075, 0.23], 
        box,
        0.3,
        [0.0, 0.0, 0.0, 1.0],
        0.2),
    place_object(ObjectInstance).

create_coffee_on_table :-
    select_surface([2.275, 1.1, 0.47],_),
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Magicokaffetypfamilycappuccino',
        0.6,
        ['map', _, [2.275, 1.1, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.0551572740078, 0.113258674741, 0.134232342243], 
        box,
        0.5,
        [0.0, 0.0, 0.0, 1.0],
        0.18),
    place_object(ObjectInstance).

create_unknown_on_table :-
    select_surface([2.275, 0.65, 0.47],_),
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Vollmilch',
        0.3,
        ['map', _, [2.275, 0.65, 0.47],[0.0, 0.0, -0.28, 0.96]],
        0.05, 
        ObjectInstance,
        [0.09, 0.06, 0.12], 
        box,
        0.4,
        [0.0, 0.0, 0.0, 1.0],
        0.3),
    place_object(ObjectInstance).