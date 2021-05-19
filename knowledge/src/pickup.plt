:- begin_tests(pickup).

:- use_module(library('test')).

:- use_module('pickup.pl').

:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


test(next_object_best_object) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, _, _, _]),
    next_object_(BestObj),
    assert_equals(BestObj, Bowl1).

test(next_object_no_source_surfaces) :-
    setup_suturo_test_source_surfaces([]),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, _, _, _]),
    next_object_(BestObj),
    assert_equals(BestObj, noSourceSurfaces).

test(next_object_no_objects_on_source_surfaces) :-
    setup_suturo_test_source_surfaces([ground]),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, _, _, _]),
    next_object_(BestObj),
    assert_equals(BestObj, noObjectsOnSourceSurfaces).


:- end_tests(pickup).
