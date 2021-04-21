:- module(beliefstate,
    [
      new_perceived_at/3,
      belief_object_at_location/3,
      belief_class_of/2,
      hsr_belief_at_update/2,
      merge_object_into_group/1,
      group_target_objects/0,
      group_shelf_objects/0,
      group_table_objects/0,
      group_objects_at/1,
      group_objects/1,
      group_mean_pose/3,
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
        retract_all_planning/1
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).


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

group_target_objects :-
    all_objects_on_target_surfaces(Objs),
    group_objects(Objs).

% Returns always true if the bucket is target.
group_target_objects :-
    all_target_surfaces(Surfaces),
    member(Surface, Surfaces),
    is_bucket(Surface).

group_shelf_objects :-
    all_objects_in_whole_shelf(Objs),
    group_objects(Objs).

group_table_objects :-
    all_objects_on_tables(Objs),
    group_objects(Objs).

group_objects_at([X,Y,Z]) :-
    Transform = ['map', _, [X,Y,Z], [0,0,1,0]],
    hsr_existing_object_at(Transform, 0.05, Obj),
    find_supporting_surface(Obj, Surface),
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
    find_supporting_surface(Member, Surface),
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
    writeln("color"),
    context_speech_sort_by_color(Source, Target, Context),
    allowed_class_distance(MaxDist),
    Distance is MaxDist + 1,
    assert_distance(Source, Distance, Context),
    !.

most_related_object(Source, Target):-
    same_size(Source, Target),
    writeln("size"),
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
    all_objects_on_target_surfaces(Objs),
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
    all_objects_on_target_surfaces(Objects),
    member(Target, Objects),
    triple(Source, hsr_objects:'colour', Color),
    triple(Target, hsr_objects:'colour', Color).
    
same_size(Source, Target):-
    all_objects_on_target_surfaces(Objects),
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
    find_supporting_surface(RefObject, Surface),
    forall(triple(Object, supposedSurface, _), tripledb_forget(Object, supposedSurface, _)),
    tell(triple(Object, supposedSurface, Surface)),
    forall(triple(Object, refObject, _), tripledb_forget(Object, refObject, _)),
    tell(triple(Object, refObject, RefObject)).

% OtherObjects returns a list of all the objects, that one day 
% would be put on Surface.
objects_on_same_surface_in_future(Surface, OtherObjects) :-
    objects_on_surface(AlreadyPlacedObjects, Surface),
    all_objects_on_source_surfaces(SourceObjects1),
    all_objects_in_gripper(SourceObjects2),
    append(SourceObjects1, SourceObjects2, SourceObjects),
    writeln(SourceObjects),
    findall(Obj,
    (
        member(Obj, SourceObjects),
        writeln(Obj),
        object_most_similar_surface(Obj, Surfacet),
        writeln(Surfacet)
    ),
        FutureObjects),
    append(AlreadyPlacedObjects, FutureObjects, OtherObjectsUnsorted),
    predsort(compareLogicalDistances, OtherObjectsUnsorted, OtherObjects).

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

next_empty_surface(Surface) :-
    all_target_surfaces(Surfaces),
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
assert_object_supposed_surface(Object) :-
    all_target_surfaces(Surfaces),
    member(Surface, Surfaces),
    is_bucket(Surface),
    all_objects_on_source_surfaces(Objs),
    context_speech_basket(Context),
    forall(member(Obj, Objs), assert_all_planning(Obj, Surface, 0, Context, Obj)),
    assert_all_planning(Object, Surface, 0, Context, Object).

assert_object_supposed_surface(Object) :-
    writeln("Hallo4"),
    object_most_similar_surface(Object, Surface),
    writeln("Hallo5"),
    writeln(Surface),
    objects_on_same_surface_in_future(Surface, OtherObjects),
    writeln("Hallo6"),
    objects_fit_on_surface(OtherObjects, Surface, _, NotFittingObjects),
    forall(member(NotFittingObject, NotFittingObjects), retract_all_planning(NotFittingObject)),
    (   member(Object, NotFittingObjects)
        -> assert_object_new_empty_surface(Object)
        ).

% First object to be placed in case of empty target surfaces
assert_object_supposed_surface(Object) :- % to do: what happens when there already are supposedSurfaces, but the according objects are not placed yet?
    all_objects_on_target_surfaces([]),
    assert_object_new_empty_surface(Object).

assert_object_new_empty_surface(Object) :-
    next_empty_surface(Surface),
    context_speech_new_class(Context),
    assert_all_planning(Object, Surface, 0, Context, Object),
    !.

object_goal_surface_(Object, Surface, Context, RefObject) :-
    triple(Object, supposedSurface, Surface),
    triple(Object, context, Context),
    triple(Object, refObject, RefObject),
    !.

object_goal_surface_(Object, Surface, Context, RefObject) :-
    place_objects,
    assert_object_supposed_surface(Object),
    object_goal_surface_(Object, Surface, Context, RefObject),
    !.
    






    % TO DO
    % -> Run new assertions for every new object
    % -> assert distance=0 and actual surface for places objects when they are placed OR percieved at target surface
    % -> what happens when there already are supposedSurfaces, but the according objects are not placed yet?
