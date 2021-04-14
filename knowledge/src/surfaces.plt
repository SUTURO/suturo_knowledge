:- begin_tests('surfaces').

:- use_module(library('test')).
:- use_module(library('lang/terms/triple')).
:- use_module(library('spatial_comp')).

:- include(library('surfaces')).

:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


test(pose_of_shelves) :-
    pose_of_shelves(Poses),
    ShelfSurfaces = ['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece', 
        'bookshelf_clone_floor_0_piece', 'bookshelf_clone_floor_1_piece', 'bookshelf_clone_floor_2_piece'],
    findall(ExpPose, 
    (
        member(Surface, ShelfSurfaces),
        tf_lookup_transform('map', Surface, pose(Pos, Rot)),
        ExpPose = [Pos, Rot]
    ), ExpPoses),
    assert_true(same_length(Poses, ExpPoses)),
    assert_true(subset(Poses, ExpPoses)),
    assert_true(subset(ExpPoses, Poses)).

test(pose_of_tables) :-
    pose_of_tables(Poses),
    TableSurfaces = ['table_front_edge_center', 'table_clone_front_edge_center'],
    findall(ExpPose, 
    (
        member(Surface, TableSurfaces),
        tf_lookup_transform('map', Surface, pose(Pos, Rot)),
        ExpPose = [Pos, Rot]
    ), ExpPoses),
    assert_true(same_length(Poses, ExpPoses)),
    assert_true(subset(Poses, ExpPoses)),
    assert_true(subset(ExpPoses, Poses)).

test(pose_of_buckets) :-
    pose_of_buckets(Poses),
    BucketSurfaces = ['bucket_surface_center'],
    findall(ExpPose, 
    (
        member(Surface, BucketSurfaces),
        tf_lookup_transform('map', Surface, pose(Pos, Rot)),
        ExpPoses = [Pos, Rot]
    ), ExpPoses),
    assert_true(same_length(Poses, ExpPoses)),
    assert_true(subset(Poses, ExpPoses)),
    assert_true(subset(ExpPoses, Poses)).

test(table_surfaces) :-
    table_surfaces(TableSurfaces),
    ExpTableSurfaces = ['table_center', 'table_clone_center'],
    assert_true(same_length(TableSurfaces, ExpTableSurfaces)),
    assert_true(subset(TableSurfaces, ExpTableSurfaces)),
    assert_true(subset(ExpTableSurfaces, TableSurfaces)).

test(bucket_surfaces) :-
    bucket_surfaces(BucketSurfaces),
    ExpBucketSurfaces = ['bucket_center'],
    assert_true(same_length(BucketSurfaces, ExpBucketSurfaces)),
    assert_true(subset(BucketSurfaces, ExpBucketSurfaces)),
    assert_true(subset(ExpBucketSurfaces, BucketSurfaces)).

test(shelf_surfaces) :-
    shelf_surfaces(ShelfSurfaces),
    ExpShelfSurfaces = ['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece', 
        'bookshelf_clone_floor_0_piece', 'bookshelf_clone_floor_1_piece', 'bookshelf_clone_floor_2_piece'],
    assert_true(same_length(ShelfSurfaces, ExpShelfSurfaces)),
    assert_true(subset(ShelfSurfaces, ExpShelfSurfaces)),
    assert_true(subset(ExpShelfSurfaces, ShelfSurfaces)).

test(ground_surface) :-
    ground_surface(Ground),
    assert_equals(Ground, ground).

test(make_all_tables_source) :-
    setup_suturo_test_source_surfaces([]),
    make_all_surface_type_role(table, source),
    findall(SourceSurface, triple(SourceSurface, hsr_objects:'sourceOrTarget', source), SourceSurfaces),
    ExpSourceSurfaces = ['table_center', 'table_clone_center'],
    assert_true(same_length(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(ExpSourceSurfaces, SourceSurfaces)).

test(make_all_shelves_source) :-
    setup_suturo_test_source_surfaces([]),
    make_all_surface_type_role(shelf, source),
    findall(SourceSurface, triple(SourceSurface, hsr_objects:'sourceOrTarget', source), SourceSurfaces),
    ExpSourceSurfaces = ['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece', 
        'bookshelf_clone_floor_0_piece', 'bookshelf_clone_floor_1_piece', 'bookshelf_clone_floor_2_piece'],
    assert_true(same_length(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(ExpSourceSurfaces, SourceSurfaces)).

test(make_ground_source) :-
    setup_suturo_test_source_surfaces([]),
    make_all_surface_type_role(ground, source),
    findall(SourceSurface, triple(SourceSurface, hsr_objects:'sourceOrTarget', source), SourceSurfaces),
    ExpSourceSurfaces = [ground],
    assert_true(same_length(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(SourceSurfaces, ExpSourceSurfaces)),
    assert_true(subset(ExpSourceSurfaces, SourceSurfaces)).

test(make_all_shelves_target) :-
    setup_suturo_test_target_surfaces([]),
    make_all_surface_type_role(shelf, target),
    findall(TargetSurface, triple(TargetSurface, hsr_objects:'sourceOrTarget', target), TargetSurfaces),
    ExpTargetSurfaces = ['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece', 
        'bookshelf_clone_floor_0_piece', 'bookshelf_clone_floor_1_piece', 'bookshelf_clone_floor_2_piece'],
    assert_true(same_length(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(ExpTargetSurfaces, TargetSurfaces)).

test(make_all_buckets_target) :-
    setup_suturo_test_target_surfaces([]),
    make_all_surface_type_role(bucket, target),
    findall(TargetSurface, triple(TargetSurface, hsr_objects:'sourceOrTarget', target), TargetSurfaces),
    ExpTargetSurfaces = ['bucket_center'],
    assert_true(same_length(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(ExpTargetSurfaces, TargetSurfaces)).

test(make_all_tables_target) :-
    setup_suturo_test_target_surfaces([]),
    make_all_surface_type_role(table, target),
    findall(TargetSurface, triple(TargetSurface, hsr_objects:'sourceOrTarget', target), TargetSurfaces),
    ExpTargetSurfaces = ['table_center', 'table_clone_center'],
    assert_true(same_length(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(TargetSurfaces, ExpTargetSurfaces)),
    assert_true(subset(ExpTargetSurfaces, TargetSurfaces)).

test(all_objects_on_tables) :-
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    all_objects_on_tables_(Objects),
    ExpObjects = [Bowl1, Cokecan1],
    assert_true(same_length(Objects, ExpObjects)),
    assert_true(subset(Objects, ExpObjects)),
    assert_true(subset(ExpObjects, Objects)).

test(all_objects_in_whole_shelf) :-
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    all_objects_in_whole_shelf_(Objects),
    ExpObjects = [Cokecan2, Spoon1],
    assert_true(same_length(Objects, ExpObjects)),
    assert_true(subset(Objects, ExpObjects)),
    assert_true(subset(ExpObjects, Objects)).

test(all_objects_in_buckets) :-
    all_objects_in_buckets(Objects),
    ExpObjects = [],
    assert_true(same_length(Objects, ExpObjects)),
    assert_true(subset(Objects, ExpObjects)),
    assert_true(subset(ExpObjects, Objects)).

test(all_objects_on_ground) :-
    all_objects_on_ground(Objects),
    ExpObjects = [];
    assert_true(same_length(Objects, ExpObjects)),
    assert_true(subset(Objects, ExpObjects)),
    assert_true(subset(ExpObjects, Objects)).

test(pose_of_target_surfaces) :-
    pose_of_target_surfaces(Poses),
    TargetSurfaces = ['table_center', 'table_clone_center'],
    setup_suturo_test_target_surfaces(TargetSurfaces),
    findall(ExpPose, 
    (
        member(Surface, TargetSurfaces),
        tf_lookup_transform('map', Surface, pose(Pos, Rot)),
        ExpPoses = [Pos, Rot]
    ), ExpPoses),
    assert_true(same_length(Poses, ExpPoses)),
    assert_true(subset(Poses, ExpPoses)),
    assert_true(subset(ExpPoses, Poses)).

test(pose_of_source_surfaces) :-
    pose_of_source_surfaces(Poses),
    SourceSurfaces = ['table_center', 'table_clone_center'],
    setup_suturo_test_source_surfaces(SourceSurfaces),
    findall(ExpPose, 
    (
        member(Surface, SourceSurfaces),
        tf_lookup_transform('map', Surface, pose(Pos, Rot)),
        ExpPoses = [Pos, Rot]
    ), ExpPoses),
    assert_true(same_length(Poses, ExpPoses)),
    assert_true(subset(Poses, ExpPoses)),
    assert_true(subset(ExpPoses, Poses)).


test(fail) :-
    fail.

:- end_tests('surfaces').