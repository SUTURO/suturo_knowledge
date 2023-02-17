%% This module loads and creates designed container in the database.
:- module(container_creation,
	  [
	      init_containers/0
	  ]).

%% init_containers is nondet
%
% Reads all containers from the URDF and creates an
% instance of type soma:'DesignedContainere' for each
%
init_containers :-
    true. % TODO: Implement