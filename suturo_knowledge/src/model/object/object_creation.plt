:- use_module(library('rostest')).
:- use_module(library('util/suturo_test')).

:- use_module('object_creation').

:- begin_rdf_tests(
		'object_creation',
		'package://suturo_knowledge/owl/suturo.owl',
		[ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')
		]).

:- rdf_meta(test_table(+,r,-,-)).
test_table(1, soma:'Table', [map, [2,-2,1], [0,0,0,1]], [data_source(semantic_map), shape(box(3,2,1))]).

setup_test_table(Num, Table) :-
    Table = 'http://www.ease-crc.org/ont/SUTURO-test.owl#Table_TEST',
    test_table(Num, Type, Pose, Options),
    create_object(Table, Type, Pose, Options).

test(table_simple_ontop, [ forall((member(X, [0.5, 3.5]),
                                   member(Y, [-3, -1]),
                                   % Note that the distance has to be a float,
                                   % because an int and a float never unify in the last line of the test.
                                   member([Z, Distance], [[1, 0.0], [1.5, 0.5]]))),
                           setup(test_setup),
                           cleanup(test_cleanup)
                         ]) :-
    setup_test_table(1, Table),
    create_object(Obj, test:'CerealBox', [map, [X, Y, Z], [0,0,0,1]]),
    object_creation:suturo_is_ontop_of(Obj, Table, ActualDistance),
    assert_equals(Distance,ActualDistance).

test(create_object_ontop, [ forall((member(X, [0.5, 3.5]),
                                    member(Y, [-3, -1]),
                                    member(Z, [1, 1.5]))),
                            setup(test_setup),
                            cleanup(test_cleanup)
					]) :-
	setup_test_table(1, Table),
	create_object(Obj, test:'CerealBox', [map, [X, Y, Z], [0,0,0,1]]),
	assert_true(kb_call(triple(Obj, soma:isOntopOf, Table))).

test(update_object, [
         setup(test_setup),
         cleanup(test_cleanup)]) :-
	create_object(X, test:'Test', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'Test', [map, [0.04,0,0], [0,0,0,1]]),
	assert_equals(X,Y).

test(update_object_not_space, [
         setup(test_setup),
         cleanup(test_cleanup)]) :-
	create_object(X, test:'Test', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'Test', [map, [0.06,0,0], [0,0,0,1]]),
	assert_false(X == Y).

test(update_object_not_space_serve, [
         setup(test_setup),
         cleanup(test_cleanup)]) :-
    %% ignore since the shelves are not there in the test.
    ignore(challenge_info_creation:init_serve_breakfast),
	create_object(X, test:'Test', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'Test', [map, [0.06,0,0], [0,0,0,1]]),
	assert_true(X == Y).


test(update_object_not_type, [
         setup(test_setup),
         cleanup(test_cleanup)]) :-
	create_object(X, test:'TestA', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'TestB', [map, [0.04,0,0], [0,0,0,1]]),
	assert_false(X == Y).

test(test_rotated_furniture, [
         setup(test_setup),
         cleanup(test_cleanup)]) :-
	create_object(A, test:furniture, [map,
								 [0,0,0],
								 [0.0, 0.0, 0.707, 0.707]],
				  [data_source(semantic_map), shape(box(0.1, 0.5, 0.1))]),
	create_object(OA, test:obj, [map, [0.2, 0, 0.1], [0,0,0,1]]),
	object_creation:suturo_is_ontop_of(OA, A, _).

:- end_rdf_tests('object_creation').
