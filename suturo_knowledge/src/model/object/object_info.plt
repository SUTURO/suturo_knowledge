:- use_module(library('rostest')).
:- use_module(library('ros/tf/tf')).
:- use_module(library('ros/tf/tf_mongo')).

:- use_module(library('model/object/object_creation'), [create_object/4]).

:- use_module('object_info').

:- begin_rdf_tests(
       'object_info',
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

test('is_misplaced object not on target') :-
    create_object(Target, test:target, [map,[0,0,0],[0,0,0,1]], [data_source(semantic_map), shape(box(1,1,0))]),
    kb_project(holds(dul:'PhysicalObject', suturo:hasOriginLocation, Target)),
    create_object(OnTarget, test:object, [map, [0,0,1], [0,0,0,1]]),
    assert_false(is_misplaced(OnTarget)).

:- end_rdf_tests('object_info').
