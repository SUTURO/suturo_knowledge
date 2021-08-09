:- begin_tests('assignplaces').

:- use_module(library('test')).

:- use_module(library('ros/urdf/URDF')).
:- use_module(library('model/SOMA/OBJ')).

:- use_module('assignplaces.pl').



test(setup) :-
    setup_suturo_test_env,
    setup_suturo_test_surfaces,
    setup_suturo_test_objects.


test(object_goal_pose_offset_when_target_shelf) :-
    reset_goal_surfaces,
    get_suturo_test_objects([_, Cokecan1, _, _]),
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    object_goal_pose_offset_(Cokecan1, [[X, Y, Z], _], _),
    tf_lookup_transform('map', 'bookshelf_floor_0_piece', pose([ExpX, ExpY, ShelffloorHeight], _)),
    object_dimensions(Cokecan1, _, _, Height),
    ExpZ is ShelffloorHeight + Height/2 + 0.07,
    assert_true(X < ExpX + 0.2),
    assert_true(X > ExpX - 0.2),
    assert_true(Y < ExpY + 0.44),
    assert_true(Y > ExpY - 0.44),
    assert_true(Z < ExpZ + 0.01),
    assert_true(Z > ExpZ - 0.01).


test(object_goal_pose_offset_when_target_bucket) :-
    reset_goal_surfaces,
    get_suturo_test_objects([_, Cokecan1, _, _]),
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bucket_center']),
    object_goal_pose_offset_(Cokecan1, [[X, Y, Z], _], _),
    tf_lookup_transform('map', 'bucket_center', pose([ExpX, ExpY, BucketHeight], _)),
    object_dimensions(Cokecan1, _, _, Height),
    ExpZ is BucketHeight + Height/2,
    assert_true(X < ExpX + 0.01),
    assert_true(X > ExpX - 0.01),
    assert_true(Y < ExpY + 0.01),
    assert_true(Y > ExpY - 0.01),
    assert_true(Z < ExpZ + 0.01),
    assert_true(Z > ExpZ - 0.01).


:- end_tests('assignplaces').