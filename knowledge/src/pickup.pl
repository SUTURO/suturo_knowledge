:- module(pickup,
    [
      next_object_/1,
      place_objects/0
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- rdf_meta
    next_object_(?).
    

next_object_(BestObj) :-
    place_objects,
    all_objects_on_source_surfaces(Objs),
    predsort(compareDistances, Objs, SortedObjs),
    nth0(0, SortedObjs, BestObj).


next_object_(noSourceSurfaces) :-
    all_source_surfaces([]),
    writeln("You haven't declared any surfaces to be source surfaces"), !.

next_object_(noObjectsOnSourceSurfaces) :-
    all_objects_on_source_surfaces([]),
    writeln("There aren't any objects on source surfaces").

place_objects :- % to do: find a better place for this!
    hsr_existing_objects(Objs),    
    forall(member(Obj, Objs), place_object(Obj)), !.

place_objects :-
    writeln("Not all objects could have been added.").