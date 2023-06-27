:- use_module(library('rostest')).

:- use_module(library('util/reset'), [reset_user_data/0]).
:- use_module(library('model/object/object_creation'), [create_object/3]).

:- use_module('directionality').

:- begin_rdf_tests(
       'directionality',
       'package://suturo_knowledge/owl/suturo.owl',
       [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#'),
         setup(object_creation_setup)
       ]).

object_creation_setup :-
    ignore(reset_user_data).

test('sort_3_objects') :-
    create_object(A,test:obja, [reference, [0,-1,0], [0,0,0,1]]),
    create_object(B,test:objb, [reference, [0, 0,0], [0,0,0,1]]),
    create_object(C,test:objc, [reference, [0,+1,0], [0,0,0,1]]),
    sort_right_to_left(reference, [A,C,B], Result),
    assert_equals(Result, [A,B,C]).
