:- begin_tests(surfaces).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('semweb/owl_parser')).
:- use_module(library('semweb/owl')).
:- use_module(library('knowrob/knowrob')).
:- use_module(library('knowrob/computable')).
:- use_module(library('knowrob/temporal')).
:- use_module(library('knowrob/beliefstate')).
:- use_module(library('knowrob/transforms')).
:- use_module(library('knowrob/vis')).

:- use_module(library('config')).
:- use_module(library('pickup')).
:- use_module(library('object_state')).
:- use_module(library('beliefstate')).
:- use_module(library('spatial_comp')).
:- use_module(library('assignplaces')).

:- include(surfaces).

%%% SETUP PREDICATES %%%%%%%%

%% RoboCup Grocery storing
create_some_roles1:-
	make_all_shelves_target,
	make_ground_source,
	make_all_tables_source.

%% RoboCup CleanUp
create_some_roles2:-
	make_all_surface_type_role(shelf, source),
	make_all_surface_type_role(table, source).

%% Different Roles within same surface types
create_some_roles3:-
	make_all_shelves_target,
	make_all_tables_source,
	make_ground_source,
	table_surfaces(Tables),
	member(Table, Tables),
	make_role(Table, target),
	shelf_surfaces(Shelves),
	member(Shelf, Shelves),
	make_role(Shelf, source).


%%% FIND SURFACES %%%%%%%%%%%


%%%%% testing all_source_surfaces/1 and all_target_surfaces/1

%% Empty Roles
test(noSourceSurfacesInTheBeginning) :-
	all_source_surfaces(Surfaces),
	Surfaces = [].

test(noTargetSurcacesInTheBeginning) :-
	all_target_surfaces(Surfaces),
	Surfaces = [].

%% Roles set by create_some_roles
test(allSourceSurfaces1, [setup(create_some_roles1)]) :-
	all_source_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, source)).

test(allSourceSurfaces2, [setup(create_some_roles2)]) :-
	all_source_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, source)).

test(allSourceSurfaces3, [setup(create_some_roles3)]) :-
	all_source_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, source)).

test(allTargetSurfaces1, [setup(create_some_roles1)]) :-
	all_target_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, target)).

test(allTargetSurfaces2, [setup(create_some_roles2)]) :-
	all_target_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, target)).

test(allTargetSurfaces3, [setup(create_some_roles3)]) :-
	all_target_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, target)).




test(makeTablesSource) :-
	all_source_surfaces(PrevSource),
	make_surfaces_target(PrevSource),
	make_all_tables_source,
	all_source_surfaces(Surfaces),
	table_surfaces(Tables),
	length(Tables, CountTables),
	length(Surfaces, CountTables).

:- end_tests(surfaces).