:- use_module(library(rostest)).
:- use_module(library(lang/query)).

:- use_module(suturo_test).

:- begin_tests(suturo_test).

test(test_1, [setup(test_setup), cleanup(test_cleanup)]) :-
    assert_true(kb_project(triple(test, test, test))).

test(test_2) :-
    assert_false(kb_call(triple(test, test, test))).

:- end_tests(suturo_test).
