:- use_module(library('rostest')).
:- use_module(library('util/suturo_test')).

:- use_module(library('model/object/object_creation')).

:- use_module('object_info').

:- begin_rdf_tests(
		'object_info',
		'package://suturo_knowledge/owl/suturo.owl',
		[ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')
		]).

%% test(example, [
%%          setup(test_setup),
%%          cleanup(test_cleanup)
%%      ]) :-
%%     true.

setup_map(Table1,Table2) :-
    create_object(Table1, test:table, [map, [-2,0,0], [0,0,0,1]],
                  [data_source(semantic_map), shape(box(1,1,1))]),
    create_object(Table2, test:table, [map, [+2,0,0], [0,0,0,1]],
                  [data_source(semantic_map), shape(box(1,1,1))]),
    kb_project((triple(soma:'CerealBox', suturo:hasDestinationLocation, Table1),
                triple(soma:'CerealBox', suturo:hasDestinationLocation, Table2))).

test(object_pose_query, [
         setup(test_setup),
         cleanup(test_cleanup)
     ]) :-
    Pose = [map, [1.0,2.0,3.0], [0.0,0.0,0.0,1.0]],
    create_object(Obj, test:obj, Pose),
    object_pose(Obj, PoseActual),
    assert_equals(Pose, PoseActual),
    Pose2 = [world, [-1.0,0.0,0.0], [0.0,0.0,0.0,1.0]],
    object_pose(Obj, Pose2),
    object_pose(Obj, Pose2Actual),
    assert_equals(Pose2, Pose2Actual).


test(is_misplaced_true, [
         setup(test_setup),
         cleanup(test_cleanup)
     ]) :-
    setup_map(_Table1, _Table2),
    create_object(Obj,soma:'CerealBox', [map, [0,0,0], [0,0,0,1]]),
    assert_true(is_misplaced_object(Obj)).

test(is_misplaced_false, [
         setup(test_setup),
         cleanup(test_cleanup)
     ]) :-
    setup_map(_Table1, _Table2),
    create_object(Obj,soma:'CerealBox', [map, [+2,0,0.6], [0,0,0,1]]),
    assert_false(is_misplaced_object(Obj)).

test(class_bfs, [
         setup(test_setup),
         cleanup(test_cleanup)
     ]) :-
    kb_project((triple(test:superclass,test:pred,val),
                triple(test:class,rdfs:subClassOf,test:superclass)
               )),
    assert_true(object_info:class_bfs(test:class,test:pred,val)).

:- end_rdf_tests('object_info').
