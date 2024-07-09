:- use_module('has_property').
:- use_module(library(util/suturo_test)).
:- use_module(library(lang/rdf_tests), [begin_rdf_tests/2, end_rdf_tests/1]).

:- begin_rdf_tests(has_property,
                   'package://suturo_knowledge/owl/suturo.owl',
                   [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')]).

test('what_object_transitive equality', [setup(test_setup), cleanup(test_cleanup), blocked('takes ca 1000s on my laptop, but works currently')]) :-
    forall(what_object_transitive(ObjName, Class),
		   (
			   assert_true((what_object_transitive(ObjName,Class2), Class = Class2)),
			   assert_true((what_object_transitive(ObjName2,Class), ObjName = ObjName2))
		   )).
