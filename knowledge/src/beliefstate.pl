:- module(beliefstate,
    [
      new_perceived_at/3,
      belief_object_at_location/3,
      belief_class_of/2,
      hsr_belief_at_update/2,
      merge_object_into_group/1,
      group_shelf_objects/0,
      group_table_objects/0,
      group_objects_at/1,
      group_objects/1,
      group_mean_pose/3,
      all_groups_on_tablelike_surface/2,
      group_position_on_surface/3,
      group_dimensions/3,
      same_color/2,
      same_size/2,
      % Placing Objects
      assert_object_supposed_surface/1,
      object_goal_surface_/4,
        %DEBUG
        assert_object_supposed_surface/1,
        next_empty_surface/1,
        objects_fit_on_surface/4,
        objects_fit_on_surface_/2,
        objects_on_same_surface_in_future/2,
        object_most_similar_surface/2,
        compareLogicalDistances/3,
        most_related_object/2,
        most_related_class/3,
        distance_to_object/3,
        retract_all_planning/1,
        object_goal_surface/2
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- use_module(library('locations/actual_locations'), 
    [
        place_objects/0,
        objects_supported_by_surface/2,
        objects_supported_by_surfaces/2
    ]).
:- use_module(library('locations/predefined_locations'), 
    [
        object_at_predefined_location/3,
        surfaces_at_predefined_location/3
    ]).
:- use_module(library('locations/spatial_comp'),
	[
		hsr_existing_object_at/3,
        compareDistances/3,
        surface_dimensions/4
	]).
:- use_module(library('gripper/gripper_info'), [all_objects_in_gripper/1]).
:- use_module(library('locations/misplaced'), [misplaced_objects_at_predefined_location/3]).
:- use_module(library('model/environment/surfaces'),
    [
        has_table_shape/1,
        has_bucket_shape/1
    ]).

:- rdf_meta
    new_perceived_at(r,+,+,r),
    belief_object_at_location(r,+,+),
    belief_class_of(r,r),
    hsr_belief_at_update(r,r),
    merge_object_into_group(r),
    group_shelf_objects,
    group_mean_pose(r,?,?).

new_perceived_at(_, Transform, Instance) :- % The underscore being ObjType.
    hsr_existing_object_at(Transform, Instance).
    % To do:
    % This part is called when an object in perceived, that
    % is already in the knowledge base.
    % Find a way to work with that and handle all the new data
    % rather than work on the class only.
    %
    % belief_class_of needs to be fixed before it can be used.
    % belief_class_of(Instance, ObjType), !.

new_perceived_at(ObjType, Transform, Instance) :-
    belief_new_object(ObjType, Instance),
    hsr_belief_at_update(Instance, Transform).

% No groups nearby
hsr_belief_at_update(Instance, Transform) :-
    tripledb_tell(Group,rdfs:'type',hsr_objects:'Group',_,[graph=groups]),
    tell(triple(Instance, hsr_objects:'inGroup', Group)),
    tell(is_at(Instance, Transform)).

merge_object_into_group(Instance) :-
    %current_object_pose(Instance, Transform),
    is_at(Instance, Transform),
    findall(NearbyObj, (
        threshold_for_group(Threshold),
        hsr_existing_object_at(Transform, Threshold, NearbyObj)),
        [Obj|Rest]),
    triple(Obj, hsr_objects:'inGroup', WG),
    member(Other, Rest),
    forall(triple(Other,hsr_objects:'inGroup',_),tripledb_forget(Other,hsr_objects:'inGroup',_)),
    tell(triple(Other, hsr_objects:'inGroup', WG)).

group_shelf_objects :-
    all_objects_in_whole_shelf(Objs),
    group_objects(Objs).

group_table_objects :-
    all_objects_on_tables(Objs),
    group_objects(Objs).

group_objects_at([X,Y,Z]) :-
    Transform = ['map', _, [X,Y,Z], [0,0,1,0]],
    hsr_existing_object_at(Transform, 0.05, Obj),
    object_supported_by_surface(Obj, Surface),
    objects_on_surface(Objs, Surface),
    group_objects(Objs).


group_objects(Objs) :-
    member(Obj, Objs),
    %current_object_pose(Obj, [map, _, Pos, _]),
    is_at(Obj, [map, Pos, _]),
    threshold_for_group(Threshold),
    hsr_existing_object_at_thr(Pos, Threshold, NearbyObj),
    triple(Obj, hsr_objects:'inGroup', Group1),
    triple(NearbyObj, hsr_objects:'inGroup', Group2),
    not(same_as(Obj, NearbyObj)),
    not(same_as(Group1, Group2)),
    triple(Member, hsr_objects:'inGroup', Group1),
    forall(triple(Member,hsr_objects:'inGroup',_),tripledb_forget(Member,hsr_objects:'inGroup',_)),
    tell(triple(Member, hsr_objects:'inGroup', Group2)),
    not(group_objects(Objs)).


group_mean_pose(Group, Transform, Rotation) :-
    findall(X, (
        triple(Member, hsr_objects:'inGroup', Group),
        %current_object_pose(Member, [_,_,[X,_,_],_])),
        is_at(Member, [_, [X,_,_], _])),
    Xs),
    findall(Y, (
        triple(Member, hsr_objects:'inGroup', Group),
        %current_object_pose(Member, [_,_,[_,Y,_],_])),
        is_at(Member, [_, [_,Y,_], _])),
    Ys),
    findall(Z, (
        triple(Member, hsr_objects:'inGroup', Group),
        %current_object_pose(Member, [_,_,[_,_,Z],_])),
        is_at(Member, [_, [_,_,Z], _])),
    Zs),
    sumlist(Xs, Xtotal),
    sumlist(Ys, Ytotal),
    sumlist(Zs,Ztotal),
    length(Xs, L),
    Xmean is Xtotal / L,
    Ymean is Ytotal / L,
    Zmean is Ztotal / L,
    Transform = [Xmean, Ymean, Zmean],
    once(triple(Member, hsr_objects:'inGroup', Group)),
    object_supported_by_surface(Member, Surface),
    surface_pose_in_map(Surface, [_, Rotation]),
    %object_frame_name(Group, Frame),
    %object_pose_update(Group, ['map', Frame, Transform, Rotation]).
    tell(is_at(Group, ['map', Transform, Rotation])).

%% Add these predicates because they are not exported in the corresponding modules
belief_object_at_location(ObjectId, NewPose, Dmax) :-
    %object_pose(ObjectId, OldPose),
    is_at(ObjectId, OldPose),
    transform_close_to(NewPose, OldPose, Dmax).

belief_class_of(Obj, ObjType) :-
    % nothing to do if current classification matches beliefs
    has_type(Obj, ObjType), !.

% To Do! assert_temporal_part and assert_temporal_part_end were part of an old knowrob
% https://github.com/daniel86/knowrob.git
% Version: 7fa6e074a6af312bc235d5ede5d092921af61095
% under knowrob_common/prolog/knowrob/temporal.pl
% With our new version of knowrob, this produces errors.
belief_class_of(Obj, NewObjType) :-
    current_time(Now),
     ignore(once((
        % withdraw old beliefs about object type
        once(instance_of(Obj, CurrObjType)),
        subclass_of(CurrObjType, Parent),
        subclass_of(NewObjType, Parent),
        assert_temporal_part_end(Obj, rdf:type, CurrObjType, Now, belief_state)
    ))),
    assert_temporal_part(Obj, rdf:type, nontemporal(NewObjType), Now, belief_state).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Assign a surface %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%% Defining the distance of their Relationship %%%%%%%%%%%%%%%%%%%%%%%%%%

most_related_object(Source, Target) :-
    not(has_type(Source, hsr_objects:'Other')),
    most_related_class(Source, Target, Distance),
    allowed_class_distance(MaxDistance),
    MaxDistance >= Distance,
    context_speech_sort_by_class(Source, Target, Distance, Context), 
    assert_distance(Source, Distance, Context),
    !.

most_related_object(Source, Target):-
    same_color(Source, Target),
    context_speech_sort_by_color(Source, Target, Context),
    allowed_class_distance(MaxDist),
    Distance is MaxDist + 1,
    assert_distance(Source, Distance, Context),
    !.

most_related_object(Source, Target):-
    same_size(Source, Target),
    context_speech_sort_by_size(Source, Target, Context),
    allowed_class_distance(MaxDist),
    Distance is MaxDist + 2,
    assert_distance(Source, Distance, Context),
    !.


most_related_class(Source, Target, Distance) :-
    %findall([Dist, T], distance_to_object(Source, T, Dist), Distances),
    findnsols(20, [Dist, T], distance_to_object(Source, T, Dist), Distances),
    min_member([Distance, Target], Distances).
    %distance_to_object(Source, Target, Distance).

distance_to_object(Source, Target, Distance) :-
    %all_objects_on_target_surfaces(Objs),
    object_at_predefined_location(Source, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    objects_supported_by_surfaces(Surfaces, Objs),
    member(Target, Objs),
    not(same_as(Source, Target)),
    has_type(Target, TargetType),
    has_type(Source, SourceType),
    distance_of(SourceType, TargetType, Distance).
    %distance_of(SourceType, TargetType, Distance).

% in case Source and Target are of the same class,
% rdf_shortest_path/3 would return 3 instead of 1. 
% This overload fixes that.
distance_of(SourceType, TargetType, Distance) :-
    same_as(SourceType, TargetType),
    Distance = 1.

% Returns the logical distance between two classes.
distance_of(SourceType, TargetType, Distance) :-
    not(same_as(SourceType, TargetType)),
    transitive(subclass_of(SourceType, Step)), 
    transitive(subclass_of(TargetType, Step)),
    transitive(subclass_of(Step, dul:'PhysicalObject')),
    path_up(SourceType, Step, DistUp),
    path_down(Step, TargetType, DistDown),
    Distance is DistUp + DistDown.
    %rdf_shortest_path(SourceType, TargetType, Distance).


path_up(SourceType, TargetType, Distance) :-
    (same_as(SourceType, TargetType)
    -> Distance = 1
    ;
    (    
        subclass_of(SourceType, Step),
        path_up(Step, TargetType, CurrentDistance),
        Distance is CurrentDistance + 1
    )).


path_down(SourceType, TargetType, Distance) :-
    (same_as(SourceType, TargetType)
    -> Distance = 1
    ;
    (
        subclass_of(Step, SourceType),
        path_down(Step, TargetType, CurrentDistance),
        Distance is CurrentDistance + 1
    )).


same_color(Source, Target):-
<<<<<<< HEAD
=======
    %all_objects_on_target_surfaces(Objects),
>>>>>>> master
    object_at_predefined_location(Source, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    objects_supported_by_surfaces(Surfaces, Objects),
    member(Target, Objects),
    triple(Source, hsr_objects:'colour', Color),
    triple(Target, hsr_objects:'colour', Color).
    
same_size(Source, Target):-
<<<<<<< HEAD
=======
    %all_objects_on_target_surfaces(Objects),
>>>>>>> master
    object_at_predefined_location(Source, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    objects_supported_by_surfaces(Surfaces, Objects),
    member(Target, Objects),
    triple(Source, hsr_objects:'size', Size),
    triple(Target, hsr_objects:'size', Size).

assert_all_planning(Object, Surface, Distance, Context, RefObject) :-
    forall(triple(Object, supposedSurface, _), tripledb_forget(Object, supposedSurface, _)),
    tell(triple(Object, supposedSurface, Surface)),
    forall(triple(Object, refObject, _), tripledb_forget(Object, refObject, _)),
    tell(triple(Object, refObject, RefObject)),
    atom_string(ContextAtom, Context),
    assert_distance(Object, Distance, ContextAtom).

assert_distance(Object, Distance, Context) :-
    atom_number(DistanceAtom, Distance),
    forall(triple(Object, distance, _), tripledb_forget(Object, distance, _)),
    tell(triple(Object, distance, DistanceAtom)),
    atom_string(ContextAtom, Context),
    forall(triple(Object, context, _), tripledb_forget(Object, context, _)),
    tell(triple(Object, context, ContextAtom)).

retract_all_planning(Object) :-
    forall(triple(Object, distance, _), tripledb_forget(Object, distance, _)),
    forall(triple(Object, context, _), tripledb_forget(Object, context, _)),
    forall(triple(Object, supposedSurface, _), tripledb_forget(Object, supposedSurface, _)),
    forall(triple(Object, refObject, _), tripledb_forget(Object, refObject, _)).


%%%%%%%%% The relation to other Objects on same surface %%%%%%%%%%%%%%%%%%%

% Returns the supposed Surface for the Object and
% stores the surface and the distance to its RefObject in RDF.
 object_most_similar_surface(Object, Surface) :-
    most_related_object(Object, RefObject),
    %find_supporting_surface(RefObject, Surface),
    object_supported_by_surface(RefObject, Surface),
    forall(triple(Object, supposedSurface, _), tripledb_forget(Object, supposedSurface, _)),
    tell(triple(Object, supposedSurface, Surface)),
    forall(triple(Object, refObject, _), tripledb_forget(Object, refObject, _)),
    tell(triple(Object, refObject, RefObject)).

% OtherObjects returns a list of all the objects, that one day 
% would be put on Surface.
objects_on_same_surface_in_future(Surface, OtherObjects) :-
    %objects_on_surface(AlreadyPlacedObjects, Surface),
    objects_supported_by_surface(Surface, AlreadyPlacedObjects),
    surface_at_predefined_location(Surface, RoomType, FurnitureType),
    misplaced_objects_at_predefined_location(SourceObjects1, RoomType, FurnitureType),
    %all_objects_on_source_surfaces(SourceObjects1),
    all_objects_in_gripper(SourceObjects2),
    append(SourceObjects1, SourceObjects2, SourceObjects),
    findall(Obj,
    (
        member(Obj, SourceObjects),
        object_most_similar_surface(Obj, Surface)
    ),
    FutureObjects),
    append(AlreadyPlacedObjects, FutureObjects, OtherObjectsUnsorted),
    predsort(compareLogicalDistances, OtherObjectsUnsorted, OtherObjectsList),
    list_to_set(OtherObjectsList, OtherObjects).

% compares the logical Distance of two Objects to their ReferenceObject on Target-Surface based on compare/3.
compareLogicalDistances(Order, Object1, Object2) :-
    triple(Object1, distance, Dist1),
    triple(Object2, distance, Dist2),
    atom_number(Dist1, Dist1N),
    atom_number(Dist2, Dist2N),
    (Dist1N = Dist2N % prevent returning "=" to prevent predsort from deleting duplicate distances.
        -> compare(Order, 0, Dist2N)
        ; compare(Order, Dist1N, Dist2N))
    .

compareLogicalDistances(Order, Object1, _) :- % Objects without a stored distance are Objects that are already placed. They will always come first.
    not(triple(Object1, distance, _)),
    compare(Order, 0, 1),
    !.

compareLogicalDistances(Order, _, Object2) :-
    not(triple(Object2, distance, _)),
    compare(Order, 1, 0),
    !.

% Takes a list of objects and divides it into the first n objects that fit on
% the given surface and the rest.
objects_fit_on_surface(Objects, Surface, FittingObjects, NotFittingObjects) :-
    objects_fit_on_surface_(Objects, Surface),
    FittingObjects = Objects,
    NotFittingObjects = [],
    !.

objects_fit_on_surface(Objects, Surface, FittingObjects, NotFittingObjects) :-
    last(Objects,LastObject), 
    delete(Objects,LastObject, ShorterList),
    objects_fit_on_surface(ShorterList, Surface, FittingObjects, NotFittingObjectsButLast),
    append(NotFittingObjectsButLast, [LastObject], NotFittingObjects),
    !.

objects_fit_on_surface_(Objects, Surface) :-
    min_space_between_objects(Threshold1),
    Threshold = Threshold1 + 0.15,
    findall(WidthPlus,
    (
        member(Object, Objects),
        object_dimensions(Object, X,Y,_),
        (   X >= Y
            -> Width = X
            ; Width = Y  ),
        WidthPlus = Width + Threshold
    ),
        ListOfWidths),
    sumlist(ListOfWidths, Sum),
    surface_dimensions(Surface, _, SurfaceWidth, _),
    SurfaceWidth >= Sum.


object_fit_on_tablelike_surface(Object, Surface, RefObject) :-
    objects_supported_by_surface(Surface, []),
    RefObject = Object,
    !.

object_fit_on_tablelike_surface(Object, Surface, RefObject) :-
    all_groups_on_tablelike_surface(Surface, Groups),
    findall([X, Group],
    (
        member(Group, Groups),
        group_position_on_surface(Group, Surface, [X, _, _])
    ),
    GroupPositions),
    sort(GroupPositions, SortedGroupPositions),
    nth0(0, SortedGroupPositions, [_, FirstGroup]),
    object_fit_in_group(Object, FirstGroup, Surface, SortedGroupPositions, RefObject),
    !.

object_fit_in_group(Object, Group, Surface, OtherGroups, RefObject) :-
    surface_dimensions(Surface, SurfaceDepth, SurfaceWidth, _),
    group_dimensions(Group, GroupWidth, GroupDepth),  
    object_dimensions(Object, ObjectWidth, ObjectDepth, _),
    min_space_between_objects(MinSpace),
    group_position_on_surface(Group, Surface, [XGroup, _, _]),
    FreeWidth is SurfaceWidth - MinSpace - GroupWidth - MinSpace,
    FreeDepth is SurfaceDepth - (SurfaceDepth/2 - XGroup),
    ((ObjectDepth < FreeWidth, ObjectWidth < FreeDepth)
    -> (
        (nextto([_, Group], [_, NextGroup], OtherGroups)
        -> (object_fit_in_group(Object, NextGroup, Surface, OtherGroups, RefObject))
        ; (most_right_object_in_group(Group, Surface, RefObject))
    ))
    ;
    (
        (nextto([_, PreviousGroup], [_, Group], OtherGroups)
        -> (most_right_object_in_group(PreviousGroup, Surface, RefObject))
        ; (
            FreeDepth2 is SurfaceDepth - (SurfaceDepth/2 - XGroup) - GroupDepth - 2*MinSpace,
            ObjectDepth < FreeDepth2,
            RefObject = Object
        )
    ))),
    !.


most_right_object_in_group(Group, Surface, Member) :-
    has_urdf_name(Surface, SurfaceLink),
    findall([X, Object], 
    (
        triple(Object, hsr_objects:'inGroup', Group),
        object_tf_frame(Object, ObjectFrame),
        tf_lookup_transform(SurfaceLink, ObjectFrame, pose([X,_,_], _))
    ), 
    Objects),
    sort(Objects, SortedObjects),
    last(SortedObjects, [_, Member]).


all_groups_on_tablelike_surface(Surface, Groups) :-
    objects_supported_by_surface(Surface, AlreadyPlacedObjects),
    findall(Group,
    (
        member(Object, AlreadyPlacedObjects),
        triple(Object, hsr_objects:'inGroup', Group)
    ),
    GroupList),
    list_to_set(GroupList, Groups).


group_dimensions(Group, Width, Depth) :-
    findall([X, Object],
    (
        triple(Object, hsr_objects:'inGroup', Group),
        is_at(Object, [_, [X,_,_], _])
    ), 
    Xs),
    findall([Y, Object],
    (
        triple(Object, hsr_objects:'inGroup', Group),
        is_at(Object, [_, [_,Y,_], _])
    ),
    Ys),
    sort(Xs, SortedXs),
    sort(Ys, SortedYs),
    nth0(0, SortedXs, [FirstX, FirstObjectX]), last(SortedXs, [LastX, LastObjectX]),
    nth0(0, SortedYs, [FirstY, FirstObjectY]), last(SortedYs, [LastY, LastObjectY]),
    object_dimensions(FirstObjectX, FirstObjectXWidth, _, _), MinX is FirstX - FirstObjectXWidth/2,
    object_dimensions(LastObjectX, LastObjectXWidth, _, _), MaxX is LastX + LastObjectXWidth/2,
    object_dimensions(FirstObjectY, _, FirstObjectYDepth, _), MinY is FirstY - FirstObjectYDepth/2,
    object_dimensions(LastObjectY, _, LastObjectYDepth, _), MaxY is LastY + LastObjectYDepth/2,
    Width is MaxX - MinX,
    Depth is MaxY - MinY.


group_position_on_surface(Group, Surface, Position) :-
    has_urdf_name(Surface, SurfaceLink),
    findall(X,
    (
        triple(Object, hsr_objects:'inGroup', Group),
        object_tf_frame(Object, ObjectFrame),
        object_dimensions(Object, ObjectWidth, _, _),
        tf_lookup_transform(SurfaceLink, ObjectFrame, pose([XCenter,_,_], _)),
        X is XCenter + ObjectWidth/2
    ), 
    Xs),
    findall(Y,
    (
        triple(Object, hsr_objects:'inGroup', Group),
        object_tf_frame(Object, ObjectFrame),
        object_dimensions(Object, _, ObjectDepth, _),
        tf_lookup_transform(SurfaceLink, ObjectFrame, pose([_,YCenter,_], _)),
        Y is YCenter - ObjectDepth/2
    ),
    Ys),
    sort(Xs, SortedXs),
    sort(Ys, SortedYs),
    last(Xs, LastX),
    nth0(0, Ys, FirstY),
    Position = [LastX, FirstY, 0].


next_empty_surface([RoomType, FurnitureType], Surface) :-
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    predsort(compareDistances, Surfaces, SortedSurfaces),
    next_empty_surface_(SortedSurfaces, Surface).

next_empty_surface(Surface) :- %% to do
    ros_warn("There is no free surface left"),
    Surface=error.

next_empty_surface_(Surfaces, Surface) :-
    member(Surface, Surfaces),
    not(triple(_, supposedSurface, Surface)).

% In case there is a bucket, put everything in it
% To Do: Cannot handle multiple surfaces including at least one bucket right now.
%assert_object_supposed_surface(Object) :-
%    %all_target_surfaces(Surfaces),
%    not object_at_predefined_location(Object, _, _),
%    writeln("Goal is bucket"),
%    bucket_surfaces(BucketSurfaces),
%    predsort(compareDistances, BucketSurfaces, SortedSurfaces),
%    nth0(0, SortedSurfaces, Surface),
%    findall(Obj, 
%    (
%        not object_at_predefined_location(Obj, _, _)
%    ), Objs),
%    context_speech_basket(Context),
%    forall(member(Obj, Objs), assert_all_planning(Obj, Surface, 0, Context, Obj)),
%    assert_all_planning(Object, Surface, 0, Context, Object).


assert_object_supposed_surface(Object) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    nth0(0, Surfaces, Surface),
    has_table_shape(Surface),
    once((
        member(TargetSurface, Surfaces),
        once(object_fit_on_tablelike_surface(Object, TargetSurface, RefObject))
    )),
    context_speech_table(Context),
    assert_all_planning(Object, TargetSurface, 0, Context, RefObject).


assert_object_supposed_surface(Object) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    nth0(0, Surfaces, Surface),
    has_bucket_shape(Surface),
    predsort(compareDistances, Surfaces, SortedSurfaces),
    nth0(0, SortedSurfaces, TargetSurface),
    context_speech_basket(Context),
    assert_all_planning(Object, TargetSurface, 0, Context, Object).

assert_object_supposed_surface(Object) :-
    object_most_similar_surface(Object, Surface),
    objects_on_same_surface_in_future(Surface, OtherObjects),
    objects_fit_on_surface(OtherObjects, Surface, _, NotFittingObjects),
    forall(member(NotFittingObject, NotFittingObjects), retract_all_planning(NotFittingObject)),
    (   member(Object, NotFittingObjects)
        -> assert_object_new_empty_surface(Object)
        ; ros_info("All objects fit on surface")
    ).

% First object to be placed in case of empty target surfaces
assert_object_supposed_surface(Object) :- % to do: what happens when there already are supposedSurfaces, but the according objects are not placed yet?
    object_at_predefined_location(Object, RoomType, FurnitureType),
    surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType),
    objects_supported_by_surfaces(Surfaces, []),
    assert_object_new_empty_surface(Object).

assert_object_new_empty_surface(Object) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    Location = [RoomType, FurnitureType],
    next_empty_surface(Location, Surface),
    context_speech_new_class(Context),
    assert_all_planning(Object, Surface, 0, Context, Object),
    !.


object_goal_surface_(Object, Surface, Context, RefObject) :-
    triple(Object, supposedSurface, Surface),
    triple(Object, context, Context),
    triple(Object, refObject, RefObject),
    !.


object_goal_surface_(Object, Surface, Context, RefObject) :-
    ignore(place_objects),
    assert_object_supposed_surface(Object),
    object_goal_surface_(Object, Surface, Context, RefObject),
    !.


object_goal_surface(Object, Surface) :-
    object_goal_surface_(Object, Surface, _, _).





    % TO DO
    % -> Run new assertions for every new object
    % -> assert distance=0 and actual surface for places objects when they are placed OR percieved at target surface
    % -> what happens when there already are supposedSurfaces, but the according objects are not placed yet?
