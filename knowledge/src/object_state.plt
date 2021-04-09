:- use_module(library('object_state')).

:- begin_tests(object_state).

%%% ===================================== OBJECT INFO ====================================================== %%%
%test('hsr_existing_objects') :- !.

%test('place_object') :- !.


%%% ===================================== OBJECT CREATION ====================================================== %%%
%test('create_object') :-
%    assert_true(X is 31).

%%% ===================================== OBJECT VALIDATION ====================================================== %%%
%test('set_dimension_semantics') :- !.

%test('set_object_color') :- !.

%test('set_color_semantics') :- !.

test('validate_confidence') :-
    assert_false(validate_confidence(0.5, 0.5, 0.4)).

test('object_size_ok') :-
    assert_false(object_size_ok([1,2,3])),
    assert_false(object_size_ok([0.01,0.01,0.009])),
    assert_false(object_size_ok([0.5, 0.5, 0.6])).
    assert_true(object_size_ok([0.5, 0.5, 0.5])).

%test('object_type_handling') :- !.


%%% ===================================== OBJECT MANIPULATION ====================================================== %%%
%test('hsr_forget_objects') :- !.

%test('forget_objects_on_surface_') :- !.

:- end_tests(object_state).
