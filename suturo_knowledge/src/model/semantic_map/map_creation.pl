%% This module loads and creates objects from the semantic map in the database.
%
:- module(map_creation,
	  [
	      init_semantic_map/0
	  ]).

init_semantic_map :-
	init_artifacts.

init_artifacts :-
	init_components,
	init_containers,
	init_furnitures.