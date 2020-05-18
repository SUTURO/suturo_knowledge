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
      assert_object_supposed_surface/1,
      object_goal_surface/4,
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
        distance_to_object/3
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

new_perceived_at(ObjType, Transform, Instance) :-
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
    kb_create(hsr_objects:'Group', Group, _{graph:groups}),
%    owl_instance_from_class(hsr_objects:'Group', Group),
    rdf_assert(Instance, hsr_objects:'inGroup', Group),
    belief_at_update(Instance, Transform).

merge_object_into_group(Instance) :-
    current_object_pose(Instance, Transform),
    findall(NearbyObj, (
        threshold_for_group(Threshold),
        hsr_existing_object_at(Transform, Threshold, NearbyObj)),
        [Obj|Rest]),
    rdf_has(Obj, hsr_objects:'inGroup', WG),
    member(Other, Rest),
    rdf_retractall(Other, hsr_objects:'inGroup', _),
    rdf_assert(Other, hsr_objects:'inGroup', WG).

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
    current_object_pose(Obj, Transform),
    threshold_for_group(Threshold),
    hsr_existing_object_at(Transform, Threshold, NearbyObj),
    rdf_has(Obj, hsr_objects:'inGroup', Group1),
    rdf_has(NearbyObj, hsr_objects:'inGroup', Group2),
    not(rdf_equal(Obj, NearbyObj)),
    not(rdf_equal(Group1, Group2)),
    rdf_has(Member, hsr_objects:'inGroup', Group1),
    rdf_retractall(Member, hsr_objects:'inGroup', _),
    rdf_assert(Member, hsr_objects:'inGroup', Group2),
    not(group_objects(Objs)).


group_mean_pose(Group, Transform, Rotation) :-
    findall(X, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[X,_,_],_])),
    Xs),
    findall(Y, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[_,Y,_],_])),
    Ys),
    findall(Z, (
        rdf_has(Member, hsr_objects:'inGroup', Group),
        current_object_pose(Member, [_,_,[_,_,Z],_])),
    Zs),
    sumlist(Xs, Xtotal),
    sumlist(Ys, Ytotal),
    sumlist(Zs,Ztotal),
    length(Xs, L),
    Xmean is Xtotal / L,
    Ymean is Ytotal / L,
    Zmean is Ztotal / L,
    Transform = [Xmean, Ymean, Zmean],
    once(rdf_has(Member, hsr_objects:'inGroup', Group)),
    find_supporting_surface(Member, Surface),
    surface_pose_in_map(Surface, [_, Rotation]),
    object_frame_name(Group, Frame),
    object_pose_update(Group, ['map', Frame, Transform, Rotation]).

%% Add these predicates because they are not exported in the corresponding modules
belief_object_at_location(ObjectId, NewPose, Dmax) :-
    object_pose(ObjectId, OldPose),
    transform_close_to(NewPose, OldPose, Dmax).

belief_class_of(Obj, ObjType) :-
    % nothing to do if current classification matches beliefs
    kb_type_of(Obj, ObjType), !.

% To Do! assert_temporal_part and assert_temporal_part_end were part of an old knowrob
% https://github.com/daniel86/knowrob.git
% Version: 7fa6e074a6af312bc235d5ede5d092921af61095
% under knowrob_common/prolog/knowrob/temporal.pl
% With our new version of knowrob, this produces errors.
belief_class_of(Obj, NewObjType) :-
    current_time(Now),
     ignore(once((
        % withdraw old beliefs about object type
        once(rdfs_individual_of(Obj, CurrObjType)),
         rdfs_subclass_of(CurrObjType, Parent),
        rdfs_subclass_of(NewObjType, Parent),
         assert_temporal_part_end(Obj, rdf:type, CurrObjType, Now, belief_state)
    ))),
    assert_temporal_part(Obj, rdf:type, nontemporal(NewObjType), Now, belief_state).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%% Assign a surface %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


%%%%%%%%% Defining the distance of their Relationship %%%%%%%%%%%%%%%%%%%%%%%%%%

most_related_object(Source, Target) :-
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
    findall(Dist, distance_to_object(Source, _, Dist), Distances),
    min_member(Distance, Distances),
    distance_to_object(Source, Target, Distance).

distance_to_object(Source, Target, Distance) :-
    all_objects_on_target_surfaces(Objs),
    member(Target, Objs),
    not(owl_same_as(Source, Target)),
    kb_type_of(Target, TargetType),
    kb_type_of(Source, SourceType),
    distance_of(SourceType, TargetType, Distance).

% in case Source and Target are of the same class,
% rdf_shortest_path/3 would return 3 instead of 1. 
% This overload fixes that.
distance_of(SourceType, TargetType, Distance) :-
    owl_same_as(SourceType, TargetType),
    Distance = 1.

% Returns the logical distance between two classes.
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

assert_all_planning(Object, Surface, Distance, Context, RefObject) :-
    rdf_retractall(Object, supposedSurface, _),
    rdf_assert(Object, supposedSurface, Surface),
    rdf_retractall(Object, refObject, _),
    rdf_assert(Object, refObject, RefObject),
    atom_string(ContextAtom, Context),
    assert_distance(Object, Distance, ContextAtom).

assert_distance(Object, Distance, Context) :-
    atom_number(DistanceAtom, Distance),
    rdf_retractall(Object, distance, _),
    rdf_assert(Object, distance, DistanceAtom),
    atom_string(ContextAtom, Context),
    rdf_retractall(Object, context, _),
    rdf_assert(Object, context, ContextAtom).

retract_all_planning(Object) :-
    rdf_retractall(Object, distance, _),
    rdf_retractall(Object, context, _),
    rdf_retractall(Object, supposedSurface, _),
    rdf_retractall(Object, refObject, _).

%%%%%%%%% The relation to other Objects on same surface %%%%%%%%%%%%%%%%%%%

% Returns the supposed Surface for the Object and
% stores the surface and the distance to it's RefObject in RDF.
 object_most_similar_surface(Object, Surface) :-
    most_related_object(Object, RefObject),
    find_supporting_surface(RefObject, Surface),
    rdf_retractall(Object, supposedSurface, _),
    rdf_assert(Object, supposedSurface, Surface),
    rdf_retractall(Object, refObject, _),
    rdf_assert(Object, refObject, RefObject).

% Object is the reference object. OtherObjects returns a list of 
% all the objects, that one day would be put on same surface as Object.
objects_on_same_surface_in_future(Surface, OtherObjects) :-
    objects_on_surface(AlreadyPlacedObjects, Surface),
    all_objects_on_source_surfaces(SourceObjects),
    findall(Obj,
    (
        member(Obj, SourceObjects),
        object_most_similar_surface(Obj, Surface)
    ),
        FutureObjects),

    append(AlreadyPlacedObjects, FutureObjects, OtherObjectsUnsorted),
    predsort(compareLogicalDistances, OtherObjectsUnsorted, OtherObjects).

% compares the logical Distance of two Objects to their ReferenceObject on Target-Surface based on compare/3.
compareLogicalDistances(Order, Object1, Object2) :-
    rdf_has(Object1, distance, Dist1),
    rdf_has(Object2, distance, Dist2),
    atom_number(Dist1, Dist1N),
    atom_number(Dist2, Dist2N),
    compare(Order, Dist1N, Dist2N).

% Takes a list of objects and divides it into the first n objects that fit on
% the given surface and the rest.
objects_fit_on_surface(Objects, Surface, FittingObjects, NotFittingObjects) :-
    last(Objects,LastObject), 
    delete(Objects,LastObject, ShorterList),
    objects_fit_on_surface(ShorterList, Surface, FittingObjects, NotFittingObjectsButLast),
    append(NotFittingObjectsButLast, [LastObject], NotFittingObjects).

objects_fit_on_surface(Objects, Surface, FittingObjects, NotFittingObjects) :-
    objects_fit_on_surface_(Objects, Surface),
    FittingObjects = Objects,
    NotFittingObjects = [],
    !.

objects_fit_on_surface_(Objects, Surface) :-
    min_space_between_objects(Threshold),
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
    writeln("There is no free surface left"),
    Surface=error.

next_empty_surface_(Surfaces, Surface) :-
    member(Surface, Surfaces),
    not(rdf_has(_, supposedSurface, Surface)).

% In case there is a bucket, put everything in it
% To Do: Can't handle multiple surfaces including at least one bucket right now.
assert_object_supposed_surface(Object) :-
    all_target_surfaces(Surfaces),
    member(Surface, Surfaces),
    is_bucket(Surface),
    all_objects_on_source_surfaces(Objs),
    context_speech_basket(Context),
    forall(member(Obj, Objs), assert_all_planning(Obj, Surface, 0, Context, Obj)),
    assert_all_planning(Object, Surface, 0, Context, Object).

assert_object_supposed_surface(Object) :-
    object_most_similar_surface(Object, Surface),
    objects_on_same_surface_in_future(Surface, OtherObjects),
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
    assert_all_planning(Object, Surface, 0, Context, Object).

object_goal_surface(Object, Surface, Context, RefObject) :-
    rdf_has(Object, supposedSurface, Surface),
    rdf_has(Object, context, Context),
    rdf_has(Object, refObject, RefObject),
    !.

object_goal_surface(Object, Surface, Context, RefObject) :-
    assert_object_supposed_surface(Object),
    object_goal_surface(Object, Surface, Context, RefObject),
    !.
    






    % TO DO
    % -> Run new assertions for every new object
    % -> assert distance=0 and actual surface for places objects when they are placed OR percieved at target surface
    % -> what happens when there already are supposedSurfaces, but the according objects are not placed yet?
