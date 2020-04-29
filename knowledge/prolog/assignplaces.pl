:- module(assignplaces,
    [
      object_goal_pose/4, 
      object_goal_pose/3, % recommendet to be used by Planning
      object_goal_pose/2,
      object_goal_pose_offset/3
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

%% In case a reference group in the shelf is found
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface(Instance, Surface, Context, RefObject),
    not(rdf_equal(Instance, RefObject)),
    surface_pose_in_map(Surface, [_, Rotation]),
    rdf_has(RefObject, hsr_objects:'inGroup', Group),
    group_mean_pose(Group, [X,Y,Z], _),
    offsets(Offset),
    member(XOffset, Offset),
    rdf_urdf_link_collision(Surface, box(Width, _, _), _),
    %hsr_lookup_transform(map, 'iai_kitchen/shelf_left_side_piece', [LeftBorder,_,_], _), % TODO: should be left corner of target-surfaces
    %hsr_lookup_transform(map, 'iai_kitchen/shelf_right_side_piece', [RightBorder,_,_], _), % TODO: should be right corner of target-surfaces
    NewX is X - (Width / 2) + XOffset,
    NewX < (Width / 2) - 0.1,
    NewX > (Width / -2) + 0.1,
    surface_front_edge_center_frame(Surface, Frame),
    tf_transform_point(Frame, map, [NewX, 0, 0], [AbsX, _,_]),
    not(hsr_existing_object_at([map,_,[AbsX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [AbsX, Y, Z],
    !.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, Instance) :-
    object_goal_surface(Instance, Surface, Context, Instance),
    surface_pose_in_map(Surface, [[X,Y,Z], Rotation]),
    offsets(Offset),
    member(XOffset, Offset),
    NewX is X + XOffset,
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z],
    !.

object_goal_pose(Intance, [Translation, Rotation], Context, Instance) :-
    object_goal_surface(Instance, Surface, Context, Instance),
    is_bucket(Surface),
    surface_pose_in_map(Surface, [Translation, Rotation]).


object_goal_pose(_, _, "You haven't defined any target surfaces", _) :-
    all_target_surfaces([]),
    writeln("You haven't defined any target surfaces").

object_goal_pose_offset(Instance, [[X,Y,Z], Rotation],Context):-
    object_goal_pose(Instance, [[X,Y,OZ], Rotation],Context),
    object_dimensions(Instance,_,_,H),
    Z is OZ + H/2 + 0.03.
