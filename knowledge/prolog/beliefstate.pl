:- module(beliefstate,
    [
      new_perceived_at/4,
      hsr_existing_object_at/4,
      belief_object_at_location/3,
      belief_class_of/2,
      hsr_belief_at_update/2,
      merge_object_into_group/1,
      group_shelf_objects/0,
      group_table_objects/0,
      group_objects_at/1,
      group_objects/1,
      group_mean_pose/3
    ]).

:-rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).


:- rdf_meta
    new_perceived_at(r,+,+,r),
    hsr_existing_object_at(r,+,+,r),
    belief_object_at_location(r,+,+),
    belief_class_of(r,r),
    hsr_belief_at_update(r,r),
    merge_object_into_group(r),
    group_shelf_objects,
    group_mean_pose(r,?,?).

new_perceived_at(ObjType, Transform, Threshold, Instance) :-
    hsr_existing_object_at(ObjType, Transform, Threshold, Instance),
    belief_class_of(Instance, ObjType), !.

new_perceived_at(ObjType, Transform, _, Instance) :-
    belief_new_object(ObjType, Instance),
    hsr_belief_at_update(Instance, Transform).

hsr_existing_object_at(_, Transform, Threshold, Instance) :-
    rdf(Instance, rdf:type, owl:'NamedIndividual', belief_state),
    rdfs_individual_of(Instance, dul:'DesignedArtifact'),
    rdf_has(Instance, hsr_objects:'supportable', true),
    belief_object_at_location(Instance, Transform, Threshold), !.


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
