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

add_object1:-
	select_surface([-1.61029851437, 0.543939828873, 0.862352252007],_),
	create_object_at(hsr_objects:'Banana',
		['map', _, [-1.61029851437, 0.543939828873, 0.862352252007],[0.0, 0.0, 0.707106781187, 0.707106781187]],
		0.05, ObjectInstance,
		[0.0379999987781, 0.0920000001788, 0.214167177677], 
		[255.0, 0.0, 0.0, 1.0]),
	place_object(ObjectInstance).

add_object2:-
	select_surface([-1.63762760162, 0.716774463654, 0.855516195297],_),
	create_object_at(hsr_objects:'Barneysbestcrunchy',
		['map', _, [-1.63762760162, 0.716774463654, 0.855516195297],[0.0, 0.0, -0.707106781187, 0.707106781187]],
		0.05, ObjectInstance,
		[0.0370000004768, 0.0810000002384, 0.224583685398], 
		[0.0, 0.0, 255.0, 1.0]),
	place_object(ObjectInstance).

add_object3:-
	select_surface([-1.65453457832, 0.895302891731, 0.824507713318],_),
	create_object_at(hsr_objects:'Applejuice',
		['map', _, [-1.65453457832, 0.895302891731, 0.824507713318],[0.0, 0.0, -0.707106781187, 0.707106781187]],
		0.05, ObjectInstance,
		[0.0340000018477, 0.0649999976158, 0.188862502575], 
		[255.0, 255.0, 0.0, 1.0]),
	place_object(ObjectInstance).

add_some_objects:-
	add_object1,
	add_object2,
	add_object3.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% ACTUAL TESTS %%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

test(forgettingObjects, [setup(add_some_objects),
	blocked("For some reason, this test unit can not execure select_surface or anything that uses object_supportable_by_surface/2.")]) :- % todo: do for other setups.
	hsr_existing_objects(Objs),
	member(Obj, Objs),
	object_current_surface(Obj, Surface),
	objects_on_surface(O, Surface),
	not(O = []),
	forget_objects_on_surface(Surface),
	objects_on_surface([], Surface).
	

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

%%%%% testing get_surface_id_by_name/2

test(getSurfaceIDByName, [blocked("For some reason, this test unit can not find rdf_urdf_name/2.")]):-
	all_surfaces(Surfaces),
	forall(member(Surface, Surfaces), (get_surface_id_by_name(Name, Surface), rdf_urdf_name(Surface, Name))). %% Todo: case get_surface_id_by_name(ground, ground) should also be true

%%%%% testing ground_surface/1, shelf_surfaces/1 and table_surfaces/1.

test(groundSurface) :-
	ground_surface(ground).

test(shelfSurfaces) :-
	shelf_surfaces(Surfaces),
	forall(member(Surface, Surfaces), rdf_has(Surface, hsr_objects:'isSurfaceType',shelf)).

test(table) :-
	table_surfaces(Surfaces),
	forall(member(Surface, Surfaces), rdf_has(Surface, hsr_objects:'isSurfaceType',table)).

test(areAllSurfacesTablesShelvesAndGroundAndBasket) :-
	ground_surface(G),
	GList = [G],
	shelf_surfaces(S),
	table_surfaces(T),
	bucket_surfaces(B),
	append(GList, S, Part1),
	append(Part1, T, Part2),
	append(Part2, B, AllDefinedSurfaces),
	length(AllDefinedSurfaces, Count),
	all_surfaces(AllSurfaces),
	length(AllSurfaces, Count).


test(makeTablesSource) :-
	all_source_surfaces(PrevSource),
	make_surfaces_target(PrevSource),
	make_all_tables_source,
	all_source_surfaces(Surfaces),
	table_surfaces(Tables),
	length(Tables, CountTables),
	length(Surfaces, CountTables).

test(fail) :-
    fail.



:- end_tests(surfaces).
