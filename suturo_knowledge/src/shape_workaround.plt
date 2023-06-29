:- use_module('shape_workaround').
:- use_module(library(util/suturo_test)).
:- use_module(library(lang/rdf_tests), [begin_rdf_tests/2, end_rdf_tests/1]).

:- use_module(library(model/object/object_creation), [create_object/4]).

:- begin_rdf_tests(shape_workaround,
                   'package://suturo_knowledge/owl/suturo.owl',
                   [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')]).

test(variable, [fail]) :-
    shape_workaround:is_valid_shape(_Variable).

test(box_true) :-
    shape_workaround:is_valid_shape(box(1,2,3)).

test(box_2, [fail]) :-
    shape_workaround:is_valid_shape(box(1,2)).

test(mesh_true) :-
    shape_workaround:is_valid_shape(mesh('path', [1,1,1])).

test(sphere_true) :-
    shape_workaround:is_valid_shape(sphere(1)).

test(cylinder_true) :-
    shape_workaround:is_valid_shape(cylinder(1,2)).

test('create_and_ask', [setup(test_setup), cleanup(test_cleanup)]) :-
    Shape = box(2.0,3.0,4.0),
    create_object(Obj, test:obj, [map, [0,0,0], [0,0,0,1]], [shape(Shape)]),
    object_shape_workaround(Obj, _Frame, ShapeTerm, _Pose, _Material),
    assert_equals(Shape, ShapeTerm).

:- end_rdf_tests(shape_workaround).
