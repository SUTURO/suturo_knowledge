:- begin_tests('beliefstate').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('lang/query')).
:- use_module(library('model/OWL')).

:- use_module(library('test')).
:- use_module(library('lang/terms/transitive')).
:- use_module(library('gripper')).

:- use_module('beliefstate.pl').



test(setup) :-
    setup_suturo_test_env,
    setup_suturo_test_surfaces,
    setup_suturo_test_objects.

test(object_most_similar_surface) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, _, _]),
    object_most_similar_surface(Cokecan1, Surface),
    assert_equals(Surface, 'bookshelf_floor_2_piece').

test(most_related_object) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, Cokecan2, _]),
    most_related_object(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(most_related_class) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, Cokecan2, _]),
    most_related_class(Cokecan1, Target, _),
    assert_equals(Target, Cokecan2).

test(same_color) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, Cokecan2, _]),
    same_color(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(same_size) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, Cokecan2, _]),
    same_size(Cokecan1, Target),
    assert_equals(Target, Cokecan2).

test(object_goal_surface_with_shelf_target) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([_, Cokecan1, Cokecan2, _]),
    object_goal_surface_(Cokecan1, Surface, _, Context),
    assert_equals(Surface, 'bookshelf_floor_2_piece'),
    assert_equals(Context, Cokecan2).

test(objects_on_same_surface_in_future) :-
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    get_suturo_test_objects([Bowl1, Cokecan1, _, _]),
    objects_on_same_surface_in_future(Cokecan1, OtherObjects),
    ExpObjects = [Cokecan1, Bowl1],
    assert_true(same_length(OtherObjects, ExpObjects)),
    assert_true(subset(OtherObjects, ExpObjects)),
    assert_true(subset(ExpObjects, OtherObjects)).
    

:- end_tests('beliefstate').
