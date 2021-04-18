:- begin_tests('beliefstate').


:- use_module(library('test')).
:- use_module(library('lang/terms/transitive')).
:- use_module(library('gripper')).
:- include(library('beliefstate')).


:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


test(object_most_similar_surface) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    object_most_similar_surface(Cokecan1, Surface),
    assert_equals(Surface, 'bookshelf_floor_2_piece').

test(most_related_object) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    most_related_object(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(most_related_class) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    most_related_class(Cokecan1, Target, Distance),
    assert_equals(Target, Cokecan2).

test(same_color) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    same_color(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(same_size) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    same_size(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(object_goal_surface_with_shelf_target) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    object_goal_surface_(Cokecan1, Surface, RefObject, Context),
    assert_equals(Surface, 'bookshelf_floor_2_piece'),
    assert_equals(Context, Cokecan2).

test(objects_on_same_surface_in_future) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    objects_on_same_surface_in_future(Cokecan1, OtherObjects),
    assert_true(OtherObjects == [Cokecan1, Bowl1]).


test(fail) :-
    fail.

:- end_tests('beliefstate').
