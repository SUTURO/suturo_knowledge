:- use_module(library('rostest')).
:- use_module(library('ros/tf/tf')).
:- use_module(library('ros/tf/tf_mongo')).

:- use_module('object_creation').

:- begin_rdf_tests(
		'object_creation',
		'package://suturo_knowledge/owl/suturo.owl',
		[ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#'),
		  setup(object_creation_setup),
		  cleanup(object_creation_cleanup)
		]).

object_creation_setup :-
	tf_mng_drop,
	tf_logger_enable.

object_creation_cleanup :-
	tf_logger_disable,
	tf_mng_drop.

:- rdf_meta(test_table(+,r,-,-)).
test_table(1, soma:'Table', [map, [2,-2,1], [0,0,0,1]], [data_source(semantic_map), shape(box(3,2,1))]).

setup_test_table(Num, Table) :-
    Table = 'http://www.ease-crc.org/ont/SUTURO-test.owl#Table_TEST',
    test_table(Num, Type, Pose, Options),
    create_object(Table, Type, Pose, Options).

test(table_simple_ontop, [ forall((member(X, [0.5, 3.5]),
                                   member(Y, [-3, -1]),
                                   % Note that the distance has to be a float,
                                   % because an int and a float never unify in the last line of suturo_is_ontop_of.
                                   member([Z, Distance], [[1, 0.0], [1.5, 0.5]])))
                         ]) :-
    setup_test_table(1, Table),
    create_object(Obj, test:'CerealBox', [map, [X, Y, Z], [0,0,0,1]]),
    object_creation:suturo_is_ontop_of(Obj, Table, Distance).

test(create_object, [ forall((member(X, [0.5, 3.5]),
							  member(Y, [-3, -1]),
							  member(Z, [1, 1.5])))
					]) :-
	setup_test_table(1, Table),
	create_object(Obj, test:'CerealBox', [map, [X, Y, Z], [0,0,0,1]]),
	assert_true(kb_call(triple(Obj, soma:isOntopOf, Table))).

test(update_object, [cleanup(kb_unproject(
								 triple(_, suturo:hasDataSource, perception)
							 ))]) :-
	create_object(X, test:'Test', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'Test', [map, [0.09,0,0], [0,0,0,1]]),
	assert_equals(X,Y).

test(update_object_not_space, [cleanup(kb_unproject(
										   triple(_, suturo:hasDataSource, perception)
									   ))]) :-
	create_object(X, test:'Test', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'Test', [map, [0.11,0,0], [0,0,0,1]]),
	assert_false(X == Y).

test(update_object_not_type, [cleanup(kb_unproject(
										  triple(_, suturo:hasDataSource, perception)
							  ))]) :-
	create_object(X, test:'TestA', [map, [0,0,0], [0,0,0,1]]),
	create_object(Y, test:'TestB', [map, [0.09,0,0], [0,0,0,1]]),
	assert_false(X == Y).

:- end_rdf_tests('object_creation').
