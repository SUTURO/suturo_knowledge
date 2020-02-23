:- module(assignplaces,
    [
      most_related_class/3,
      most_related_object/2,
      best_place/2
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    most_related_class(-,?,?),
    most_related_object(-,?),
    best_place(-,?).


best_place(Source, Target) :-
    Target = 6. %% Todo

most_related_object(Source, Target) :-
    most_related_class(Source, Target, _).

most_related_class(Source, Target, Distance) :-
    findall(Dist, distance_to_object(Source, _, Dist), Distances),
    min_member(Distance, Distances),
    distance_to_object(Source, Target, Distance),
    allowed_class_distance(MaxDist),
    MaxDist >= Distance.

distance_to_object(Source, Target, Distance) :-
    all_objects_in_whole_shelf(Objs), %% should be used insted of hsr_existing_objects
    member(Target, Objs),
    member(Source, Objs),
    not(owl_same_as(Source, Target)),
    kb_type_of(Target, TargetType),
    kb_type_of(Source, SourceType),
    distance_of(SourceType, TargetType, Distance).

distance_of(SourceType, TargetType, Distance) :-
    owl_same_as(SourceType, TargetType),
    Distance = 1.

distance_of(SourceType, TargetType, Distance) :-
    not(owl_same_as(SourceType, TargetType)),
    rdf_shortest_path(SourceType, TargetType, Distance).
