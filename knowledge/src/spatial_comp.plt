:- begin_tests('spatial_comp').

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('db/tripledb')).
:- use_module(library('knowrob')).

:- use_module(library('config')).
:- use_module(library('pickup')).
:- use_module(library('object_state')).
:- use_module(library('beliefstate')).
:- use_module(library('surfaces')).
:- use_module(library('assignplaces')).

:- include(spatial_comp).

%%% SETUP PREDICATES %%%%%%%%



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% ACTUAL TESTS %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


test(quaternionEuler) :-
	quaternion_to_euler([0,0,0,1],[0.0,0.0,0.0]).
	%quaternion_to_euler([ 0.0002561, 1, 0.0001707, 0.0000427 ], [ 3.1412512, 0.0000853, 3.1410805 ]).


:- end_tests(spatial_comp).