:- module(assignplaces,
    [
      offsets2/1
      most_related_class/2
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(srdl2_comp, 'http://knowrob.org/kb/srdl2-comp.owl#', [keep(true)]).

:- rdf_meta
    offsets2(?).

offsets2(Offset) :-
    Offset = [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, 0.35, 0.35].

most_related_class(Source, Target, Distance) :-
    all_objects_in_whole_shelf(Objs),
    member(Target, Objs),
    kb_type_of(Target, TargetType),
    kb_type_of(Source, SourceType),
    rdf_shortest_path()