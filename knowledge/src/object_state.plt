:- use_module(library('object_state')).

:- begin_tests(object_state).


test(create_object) :-
    assert_true(X is 31).

:- end_tests(object_state).
