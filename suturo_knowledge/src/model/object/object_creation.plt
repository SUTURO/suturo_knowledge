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
test_table(1, soma:'Table', [map, [2,-2,1], [0,0,0,1]], [dataSource(semantic_map), shape(box(3,2,1))]).

setup_test_table(Num, Table) :-
    test_table(Num, Type, Pose, Options),
    create_object(Table, Type, Pose, Options).

test(table_simple_ontop, [ forall((member(X, [0.5, 1, 1.5, 2, 2.5, 3, 3.5]),
                                   member(Y, [-3, -2.5, -2, -1.5, -1]),
                                   % Note that the distance has to be a float,
                                   % because an int and a float never unify in the last line of suturo_is_ontop_of.
                                   member([Z, Distance], [[1, 0.0], [1.5, 0.5], [2, 1.0]])))
                         ]) :-
    setup_test_table(1, Table),
    create_object(Obj, soma:'CerealBox', [map, [X, Y, Z], [0,0,0,1]]),
    object_creation:suturo_is_ontop_of(Obj, Table, Distance).

:- end_rdf_tests('object_creation').
