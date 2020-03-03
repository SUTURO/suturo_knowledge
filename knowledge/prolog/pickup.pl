:- module(pickup,
    [
      next_object/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).

:- rdf_meta
    next_object(?).
    

next_object(BestObj) :-
    all_objects_on_source_surfaces(Objs),
    maplist(distance_to_robot, Objs, Distances),
    min_list(Distances, MinDistance),
    nth0(Index, Distances, MinDistance),
    nth0(Index, Objs, NearestObject),
    BestObj = NearestObject.

distance_to_robot(Obj, Distance) :-
    map_frame_name(MapFrame),
    current_object_pose(Obj, [MapFrame,_,[OX,OY,OZ],_]),
    tf_lookup_transform(map,'base_footprint',[_,[BX,BY,BZ],_]),
    DX is OX - BX,
    DY is OY - BY,
    DZ is OZ - BZ,
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.