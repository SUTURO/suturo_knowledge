:- module(pickup,
    [
      next_object/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).

:- rdf_meta
    next_object(?).
    



next_object(BestObj) :- % Todo create and use surface start/goal attribute
    all_objects_on_tables(Objs),
    maplist(distance_to_robot, Objs, Distances),
    min_list(Distances, MinDistance),
    nth0(Index, Distances, MinDistance),
    nth0(Index, Objs, NearestObject),
    BestObj = NearestObject.

distance_to_robot(Obj, Distance) :-
    object_distance(base_footprint, Obj, Distance).