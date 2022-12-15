%% This module loads and creates designed components in the database.
:- module(component_creation,
	  [
	      init_components/0
	  ]).

%% init_components is nondet
%
% Reads all components from the URDF and creates an
% instance of type soma:'DesignedContainere' for each
%
init_components :-
    true. % TODO: Implement