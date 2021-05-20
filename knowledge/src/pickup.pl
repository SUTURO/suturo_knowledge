:- module(pickup,
    [
      next_object_/1,
      place_objects/0,
      object_pose_to_grasp_from/2
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

object_pose_to_grasp_from(Object,[[XPose,YPose,0], Rotation]):-
    ask(triple(Object, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#supportedBy', Surface)),
    surface_front_edge_center_frame(Surface, SurfaceFrame),
    surface_front_edge_center_pose(Surface,[_, Rotation]),
    object_tf_frame(Object,F),
    tf_lookup_transform(SurfaceFrame, F, pose([_,Y,_],_)),
    tf_transform_point(SurfaceFrame,map, [-0.5, Y,0], [XPose,YPose,_]).
