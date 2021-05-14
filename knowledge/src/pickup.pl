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
    ros_warn("You haven't declared any surfaces to be source surfaces"), !.

next_object_(noObjectsOnSourceSurfaces) :-
    all_objects_on_source_surfaces([]),
    ros_info("There aren't any objects on source surfaces").

place_objects :- % to do: find a better place for this!
    hsr_existing_objects(Objs),    
    forall(member(Obj, Objs), place_object(Obj)), !.

place_objects :-
    ros_warn("Not all objects could have been added.").

object_pose_to_grasp_from(Object,[RelativePosition, Rotation]):-
    ask(triple(Object, hsr_objects:'supportedBy', Surface)),
    surface_front_edge_center_frame(Surface, SurfaceFrame),
    surface_front_edge_center_pose(Surface,[_, Rotation]),
    tf_lookup_transform(SurfaceFrame, Object, pose([_,Y,Z],_)),
    Ynew is Y + 0.5,
    tf_transform_point(map, SurfaceFrame, [0, -Ynew,-Z], RelativePosition).
