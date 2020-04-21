:- module(assignplaces,
    [
      object_goal_surface/4,
      object_goal_surface/2,
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

object_goal_surface(Instance, Surface) :-
    object_goal_surface(Instance, Surface, _, _).


object_goal_surface(Instance, Surface, Context, RefObject):-
    most_related_object(Instance, RefObject, Context),
    find_supporting_surface(RefObject, Surface).


%% If there is no corresponding class, take the nearest easy reachable target surface that is empty
object_goal_surface(Instance, NearestReachableSurface, Context, Self) :-
    all_target_surfaces(Surfaces),
    findall(Surface,
    (
        member(Surface, Surfaces),
        surface_pose_in_map(Surface, [[_,_,Z],_]),
        Z < 1.3 % Surfaces higher than 1.3m might be hard to reach by the HSR
    ),
        ReachableSurfaces),
    maplist(distance_to_robot, ReachableSurfaces, Distances),
    min_list(Distances, MinDistance),
    nth0(Index, Distances, MinDistance),
    nth0(Index, ReachableSurfaces, NearestReachableSurface),
    objects_on_surface([], NearestReachableSurface),
    Self = Instance,
    context_speech_new_class(Context).

%% If there is no corresponding class and there is no easy reachable target surface, 
%% take the nearest target surface (that is not easily reachable).
object_goal_surface(Instance, NearestSurface, Context, Self) :-
    all_target_surfaces(Surfaces),
    maplist(distance_to_robot, Surfaces, Distances),
    min_list(Distances, MinDistance),
    nth0(Index, Distances, MinDistance),
    nth0(Index, Surfaces, NearestSurface),
    objects_on_surface([], NearestSurface),
    Self = Instance,
    context_speech_new_class(Context).

object_goal_surface(_, _, "You haven't defined any target surfaces", _) :-
    all_target_surfaces([]),
    writeln("You haven't defined any target surfaces").

most_related_object(Source, Target, Context):-
    kb_type_of(Source, hsr_objects:'Other'),
    same_color(Source, Target),
    context_speech_sort_by_color(Source, Target, Context).

most_related_object(Source, Target, Context):-
    kb_type_of(Source, hsr_objects:'Other'),
    same_size(Source, Target),
    context_speech_sort_by_size(Source, Target, Context).

most_related_object(Source, Target, Context) :-
    most_related_class(Source, Target, Distance),
    context_speech_sort_by_class(Source, Target, Distance, Context).

most_related_class(Source, Target, Distance) :-
    findall(Dist, distance_to_object(Source, _, Dist), Distances),
    min_member(Distance, Distances),
    distance_to_object(Source, Target, Distance),
    allowed_class_distance(MaxDist),
    MaxDist >= Distance.

distance_to_object(Source, Target, Distance) :-
    all_objects_on_target_surfaces(Objs),
    member(Target, Objs),
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

same_color(Source, Target):-
    all_objects_on_target_surfaces(Objects),
    member(Target, Objects),
    rdf_has(Source, hsr_objects:'colour', Color),
    rdf_has(Target, hsr_objects:'colour', Color).
    
same_size(Source, Target):-
    all_objects_on_target_surfaces(Objects),
    member(Target, Objects),
    rdf_has(Source, hsr_objects:'size', Size),
    rdf_has(Target, hsr_objects:'size', Size).


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
    Translation = [AbsX, Y, Z] ,!.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface(Instance, Surface, Context, RefObject),
    rdf_equal(Instance, RefObject),
    surface_pose_in_map(Surface, [[X,Y,Z], Rotation]),
    offsets(Offset),
    member(XOffset, Offset),
    NewX is X + XOffset,
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z].

object_goal_pose(_, _, "You haven't defined any target surfaces", _) :-
    all_target_surfaces([]),
    writeln("You haven't defined any target surfaces").

object_goal_pose_offset(Instance, [[X,Y,Z], Rotation],Context):-
    object_goal_pose(Instance, [[X,Y,OZ], Rotation],Context),
    object_dimensions(Instance,_,_,H),
    Z is OZ + H/2 + 0.03.
