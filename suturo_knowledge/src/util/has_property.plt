:- use_module('has_property').
:- use_module(library(util/suturo_test)).
:- use_module(library(lang/rdf_tests), [begin_rdf_tests/2, end_rdf_tests/1]).

:- begin_rdf_tests(has_property,
                   'package://suturo_knowledge/owl/suturo.owl',
                   [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% what_object
%test('what_object', [setup(test_setup), cleanup(test_cleanup)]):-



%% what_object_transitive
%%
test('what_object_transitive equality', [setup(test_setup), cleanup(test_cleanup), blocked('takes ca 1000s on my laptop, but works currently')]) :-
    forall(what_object_transitive(ObjName, Class),
		   (
			   assert_true((what_object_transitive(ObjName,Class2), Class = Class2)),
			   assert_true((what_object_transitive(ObjName2,Class), ObjName = ObjName2))
		   )).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% is_fragile
%%
test('is_fragile true', [setup(test_setup), cleanup(test_cleanup)]) :-
	is_fragile('milk glass')

test('is_fragile false', [setup(test_setup), cleanup(test_cleanup)]) :-
	is_fragile('bed').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% is_perishable
%%
test('is_perishable true', [setup(test_setup), cleanup(test_cleanup)]) :-
	is_perishable('apple juice').

test('is_perishable false', [setup(test_setup), cleanup(test_cleanup)]) :-
	is_perishable('metal bowl').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% grasp_pose
%%
test('grasp_pose' above, [setup(test_setup), cleanup(test_cleanup)]) :-
	grasp_pose('spoon').

test('grasp_pose' side, [setup(test_setup), cleanup(test_cleanup)]) :-
	grasp_pose('bowl').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% is_light_or_heavy
%%
test('is_light_or_heavy' heavy, [setup(test_setup), cleanup(test_cleanup)]) :-
	assert_equals(is_light_or_heavy('bowl'), 'heavy').

test('is_light_or_heavy' light, [setup(test_setup), cleanup(test_cleanup)]) :-
	assert_equals(is_light_or_heavy('dishwashertab'), 'light').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% save_person_data and call_person_data
%%
%% alles passende eingaben

test('save_person_data' all, [setup(test_setup), cleanup(test_cleanup)]) :-
	Name1 = 'Nicolas',
	Drink1 = 'cacao',
	Interest1 = 'tennis',
	Profession1 = 'bartender',
	save_person_data(1.0, Name1, Drink1, Hobby1, Profession1),
	assert_true(call_person_data(1.0, Name, Drink, Interest, Profession), 
				Name = Name1,
				Drink = Drink1,
				Interest = Interest1,
				Profession = Profession1
				).
% eine/mehrere unpassende eingabe
test('save_person_data' , [setup(test_setup), cleanup(test_cleanup)]) :-
	save_person_data(1.0, _, 'cacao', 'tennis', 'bartender').


% option: daten überschrieben


% leere eingaben? 
test('save_person_data' , [setup(test_setup), cleanup(test_cleanup)]) :-
	Name1 = 'Nicolas',
	Drink1 = 'cacao',
	Interest1 = 'tennis',
	Profession1 = 'bartender',
	save_person_data(1.0, Name1, Drink1, Hobby1, Profession1),
	assert_true(call_person_data(1.0, Name, Drink, Interest, Profession), 
				Name = Name1,
				Drink = Drink1,
				Interest = Interest1,
				Profession = Profession1
				).


