:- use_module(library('rostest')).

:- use_module('semantic_similarity').

:- begin_rdf_tests(
		'semantic_similarity',
		'package://suturo_knowledge/owl/suturo.owl',
		[ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')
		]).

setup_grandparent :-
    kb_project((
                      subclass_of(test:a,test:b),
                      subclass_of(test:b,test:c)
              )).

setup_cousin :-
    setup_grandparent,
    kb_project((
                      subclass_of(test:sibling,test:b),
                      subclass_of(test:uncle,test:c),
                      subclass_of(test:cousin,test:uncle)
              )).

test('self is one') :-
    wu_palmer_similarity(x,x,Sim),
    assert_equals(Sim, 1).

test('direct ancestor is more similar than indirect ancestor',
     [ fixme('wu_palmer_similarity(test:a,test:c,Sim2) fails since it can`t find an lcs for the topmost class')
     ]) :-
    setup_grandparent,
    wu_palmer_similarity(test:a,test:b,Sim1),
    wu_palmer_similarity(test:a,test:c,Sim2),
    assert_true(Sim1 > Sim2).

test('sibling class is more similar than cousin') :-
    setup_cousin,
    wu_palmer_similarity(test:a,test:sibling,Sim1),
    wu_palmer_similarity(test:a,test:cousin,Sim2),
    assert_true(Sim1 > Sim2).

test('sort self, sibling, cousin, not an object') :-
    setup_cousin,
    kb_project((has_type(test:a_self, test:a),
                has_type(test:sibling_i, test:sibling),
                has_type(test:cousin_i, test:cousin))),
    sort_by_similarity(test:a_self, [none,test:sibling_i,test:a_self,test:cousin_i], Sorted),
    assert_equals(Sorted, [test:a_self,test:sibling_i,test:cousin_i,none]).

:- end_tests('semantic_similarity').
