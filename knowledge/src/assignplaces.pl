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

object_goal_pose(Instance, [Translation, Rotation], Context, Instance) :-
    object_goal_surface_(Instance, Surface, Context, Instance),
    is_bucket(Surface),
    surface_pose_in_map(Surface, [Translation, Rotation]),
    !.

%% In case a reference group in the shelf is found
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface_(Instance, Surface, Context, RefObject),
    not(rdf_equal(Instance, RefObject)),
    surface_pose_in_map(Surface, [_, Rotation]),
    triple(RefObject, hsr_objects:'inGroup', Group),
    group_mean_pose(Group, [GroupX,GroupY,GroupZ], _),
    tf_transform_point(map, Surface, [GroupX,GroupY,GroupZ], [_, GroupYOnS, _]),
    offsets(Offset),
    member(YOffset, Offset),
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF, Surface, box(Depth, Width, _), _), % get surface dimensions
    NewY is (GroupYOnS + YOffset),
    NewY < (Width / 2) - 0.1,
    NewY > (Width / -2) + 0.1,
    NewX is - Depth / 2 + 0.03,
    tf_transform_point(Surface, map, [NewX, NewY, 0], [AbsX, AbsY,AbsZ]),
    object_dimensions(Instance,_,_,ObjHeight),
    AbsZOffset is AbsZ + ObjHeight/2 + 0.07,
    not(hsr_existing_object_at_thr([AbsX, AbsY, AbsZOffset], 0.15)),
    Translation = [AbsX, AbsY, AbsZOffset],
    !.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, Instance) :-
    rosinfo("object_goal_pose created new Group"),
    object_goal_surface_(Instance, Surface, Context, Instance),
    surface_pose_in_map(Surface, [[SX,SY,SZ], Rotation]),
    tf_transform_point(map, Surface, [SX,SY,SZ], [ _, YOnS,_]),  
    offsets(Offset),
    member(YOffset, Offset),
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF, Surface, box(Depth, Width, _), _), % get surface dimensions
    NewYOnS is YOnS + YOffset,
    NewYOnS < (Width / 2) - 0.1,
    NewYOnS > (Width / -2) + 0.1,
    NewXOnS is - Depth / 2 + 0.03,
    tf_transform_point(Surface, map, [NewXOnS, NewYOnS, 0], [AbsX, AbsY,AbsZ]),
    object_dimensions(Instance,_,_,ObjHeight),
    AbsZOffset is AbsZ + ObjHeight/2 + 0.07,
    not(hsr_existing_object_at_thr([AbsX, AbsY,AbsZOffset], 0.15)),
    Translation = [AbsX, AbsY,AbsZOffset],
    !.

object_goal_pose(_, _, "You haven't defined any target surfaces", _) :-
    all_target_surfaces([]),
    roswarn("You haven't defined any target surfaces").

% deprecated. Use object_goal_pose instead.
object_goal_pose_offset_(Instance, [[X,Y,Z], Rotation],Context):-
    place_objects,
    object_goal_pose(Instance, [[X,Y,Z], Rotation],Context).
