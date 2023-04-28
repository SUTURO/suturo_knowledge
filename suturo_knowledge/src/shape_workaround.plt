:- begin_tests(shape_workaround).
:- use_module('shape_workaround').

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

:- end_tests(shape_workaround).
