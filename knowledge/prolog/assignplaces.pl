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
    surface_pose_in_map(Surface, [[XWOOffset,Y,Z], Rotation]),
    X is XWOOffset + 0.0,
    % X is XWOOffset - 0.05, for 5cm closer to edge
    % X is XWOOffset + 0.05, for 5cm further away from edge
    % - 0.15 is the outer edge of the current bucket
    Translation = [X,Y,Z],
    !.

%% In case a reference group in the shelf is found
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface_(Instance, Surface, Context, RefObject),
    not(rdf_equal(Instance, RefObject)),
    surface_pose_in_map(Surface, [_, Rotation]),
    rdf_has(RefObject, hsr_objects:'inGroup', Group),
    group_mean_pose(Group, [GroupX,GroupY,GroupZ], _),
    urdf_frame(Surface, Frame),
    tf_transform_point(map, Frame, [GroupX,GroupY,GroupZ], [_, GroupYOnS, _]),
    offsets(Offset),
    member(YOffset, Offset),
    rdf_urdf_link_collision(Surface, box(Depth, Width, _), _), % get surface dimensions
    NewY is (GroupYOnS + YOffset),
    NewY < (Width / 2) - 0.1,
    NewY > (Width / -2) + 0.1,
    NewX is - Depth / 2 + 0.03,
    tf_transform_point(Frame, map, [NewX, NewY, 0], [AbsX, AbsY,AbsZ]),
    not(hsr_existing_object_at_thr([AbsX, AbsY, AbsZ], 0.15)),
    Translation = [AbsX, AbsY, AbsZ],
    !.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, Instance) :-
    write("object_goal_pose new Group"),
    object_goal_surface_(Instance, Surface, Context, Instance),
    urdf_frame(Surface, Frame),
    surface_pose_in_map(Surface, [[SX,SY,SZ], Rotation]),
    tf_transform_point(map, Frame, [SX,SY,SZ], [ _, YOnS,_]),  
    offsets(Offset),
    member(YOffset, Offset),
    rdf_urdf_link_collision(Surface, box(Depth, Width, _), _), % get surface dimensions
    NewYOnS is YOnS + YOffset,
    NewYOnS < (Width / 2) - 0.1,
    NewYOnS > (Width / -2) + 0.1,
    NewXOnS is - Depth / 2 + 0.03,
    tf_transform_point(Frame, map, [NewXOnS, NewYOnS, 0], [AbsX, AbsY,AbsZ]),
    not(hsr_existing_object_at_thr([AbsX, AbsY,AbsZ], 0.15)),
    Translation = [AbsX, AbsY,AbsZ],
    !.

object_goal_pose(_, _, "You haven't defined any target surfaces", _) :-
    all_target_surfaces([]),
    writeln("You haven't defined any target surfaces").


% deprecated. Use object_goal_pose instead. TODO reanabled for M1
object_goal_pose_offset_(Instance, [[XR,YR,ZR], Rotation],Context):-
    place_objects,
    object_goal_pose(Instance, [[X,Y,Z], Rotation],Context),
    object_dimensions(Instance,_,_,ObjHeight),
    XR is X + 0,
    YR is Y + 0,
    ZR is Z + ObjHeight/2 + 0.07.

