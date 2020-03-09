:- module(assignplaces,
    [
      object_goal_surface/4,
      object_goal_pose/4,
      object_goal_pose/3,
      object_goal_pose/2,
      object_goal_pose_offset/3
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://knowrob.org/kb/robocup.owl#', [keep(true)]).

:- rdf_meta
    most_related_class(-,?,?),
    most_related_object(-,?).


object_goal_surface(Instance, Surface, Context, RefObject):-
    most_related_object(Instance, RefObject, Context),
    object_current_surface(RefObject, Surface).


%******************************
%% If there is no corresponding class, take some shelf in the middle
object_goal_surface(Instance, SurfaceLink, Context, Self) :-
    (shelf_floor_at_height(0.9, SurfaceLink);
    shelf_floor_at_height(0.6, SurfaceLink)),
    objects_on_surface([], SurfaceLink),
    Self = Instance,
    context_speech_new_class(Context).

%% If middle shelves also occupied, take rest (lowest level first). WARNING: HSR may not be able to reach upper levels
object_goal_surface(Instance, SurfaceLink, Context, Self) :-
    big_shelf_surfaces(ShelfFloors),
    member(SurfaceLink,ShelfFloors),
    objects_on_surface([], SurfaceLink),
    Self = Instance,
    context_speech_new_class(Context).
%***********************************

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
    hsr_lookup_transform(map, 'iai_kitchen/shelf_left_side_piece', [LeftBorder,_,_], _),
    hsr_lookup_transform(map, 'iai_kitchen/shelf_right_side_piece', [RightBorder,_,_], _),
    NewX is X + XOffset,
    NewX < LeftBorder - 0.1,
    NewX > RightBorder + 0.1,
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z] ,!.

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

object_goal_pose_offset(Instance, [[X,Y,Z], Rotation],Context):-
    object_goal_pose(Instance, [[X,Y,OZ], Rotation],Context),
    object_dimensions(Instance,_,_,H),
    Z is OZ + H/2 + 0.03.

