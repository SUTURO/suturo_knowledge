:- module(pickup,
    [
      next_object/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).

:- rdf_meta
    next_object(?).
    

next_object(BestObj) :-
    place_objects,
    all_objects_on_source_surfaces(Objs),
    maplist(distance_to_robot, Objs, Distances),
    min_list(Distances, MinDistance),
    nth0(Index, Distances, MinDistance),
    nth0(Index, Objs, NearestObject),
    BestObj = NearestObject.


next_object(noSourceSurfaces) :-
    all_source_surfaces([]),
    writeln("You haven't declared any surfaces to be source surfaces"), !.

next_object(noObjectsOnSourceSurfaces) :-
    all_objects_on_source_surfaces([]),
    writeln("There aren't any objects on source surfaces").

place_objects :-
    hsr_existing_objects(Objs),
    forall(member(Obj, Objs), place_object(Obj)).
