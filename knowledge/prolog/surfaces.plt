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



%%% FIND SURFACES %%%%%%%%%%%

test(noSourceSurfacesInTheBeginning) :-
	all_source_surfaces(Surfaces),
	Surfaces = [].

test(noTargetSurcacesInTheBeginning) :-
	all_target_surfaces(Surfaces),
	Surfaces = [].

test(allSourceSurfaces1) :-
	all_source_surfaces(Surfaces),
	forall(member(Surface, Surfaces), get_surface_role(Surface, source)).

test(makeTablesSource) :-
	all_source_surfaces(PrevSource),
	make_surfaces_target(PrevSource),
	make_all_tables_source,
	all_source_surfaces(Surfaces),
	table_surfaces(Tables),
	length(Tables, CountTables),
	length(Surfaces, CountTables).

:- end_tests(surfaces).