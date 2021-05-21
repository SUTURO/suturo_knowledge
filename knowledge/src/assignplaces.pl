:- module(assignplaces,
    [
      object_goal_pose/4, 
      object_goal_pose/3, % recommendet to be used by Planning
      object_goal_pose/2,
      object_goal_pose_offset_/3
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- rdf_meta
    most_related_class(-,?,?),
    most_related_object(-,?).



%******************** GOAL POSE **********************

object_goal_pose(Instance, [Translation, Rotation]) :-
    object_goal_pose(Instance, [Translation, Rotation], _).

object_goal_pose(Instance, [Translation, Rotation], Context) :-
    object_goal_pose(Instance, [Translation, Rotation], Context, _).

object_goal_pose(Instance, [Translation, Rotation], Context, RefInstance) :-
    has_table_shape(Table),
    has_urdf_name(Table,Name),
    sub_string(Name,_,_,_,bucket),
    surface_center_pose(Table,[Translation, Rotation]).


% TODO Rework the offsets
object_goal_pose_offset_(Instance, [[XR,YR,ZR], Rotation],Context):-
    place_objects,
    object_goal_pose(Instance, [[X,Y,Z], Rotation],Context),
    object_dimensions(Instance,_,_,ObjHeight),
    XR is X + 0,
    YR is Y + 0,
    ZR is Z + ObjHeight/2 + 0.07.

