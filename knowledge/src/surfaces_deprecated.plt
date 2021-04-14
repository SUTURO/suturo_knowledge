:- use_module(library('db/tripledb_tests')).

:- use_module(library('surfaces')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('db/tripledb'),[tripledb_load/2, ros_package_iri/2, tripledb_forget/3]).
:- use_module(library('lang/terms/triple')).
:- use_module(library('model/RDFS')).
:- use_module(library('knowrob')).
:- use_module(library('rostest.pl')).

:- use_module(library('config')).
:- use_module(library('pickup')).
:- use_module(library('object_state')).
:- use_module(library('beliefstate')).
:- use_module(library('spatial_comp')).
:- use_module(library('assignplaces')).

:- begin_tripledb_tests(
      	'surfaces',
       	'package://knowledge/owl/testing.owl',
       	[ namespace('http://www.semanticweb.org/suturo/ontologies/2021/0/testing#')]
 ).



%%% SETUP PREDICATES %%%%%

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
	forall(member(Surface, Surfaces), triple(Surface, hsr_objects:'isSurfaceType',shelf)).

test(table) :-
	table_surfaces(Surfaces),
	forall(member(Surface, Surfaces), triple(Surface, hsr_objects:'isSurfaceType',table)).

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



cleanup_roles :-
	forall(triple(Object,hsr_objects:'sourceOrTarget',Role), tripledb_forget(Object,hsr_objects:'sourceOrTarget',Role)).


test(setup) :-
	tell(triple(test:'Shelffloor1',hsr_objects:'isSurfaceType', shelf)),
	tell(triple(test:'Shelffloor2', hsr_objects:'isSurfaceType', shelf)),
	tell(triple(test:'Tabletop1',hsr_objects:'isSurfaceType',table)),
	tell(triple(test:'Tabletop2',hsr_objects:'isSurfaceType',table)),
	tell(triple(test:'BucketOpening1',hsr_objects:'isSurfaceType',bucket)),
	tell(triple(test:'BucketOpening2',hsr_objects:'isSurfaceType',bucket)),
	tell(triple(test:'Chips1',hsr_objects:'supportedBy',test:'Tabletop1')),
	tell(triple(test:'IceTea1',hsr_objects:'supportedBy',test:'Tabletop1')),
	tell(triple(test:'CoffeeMug1',hsr_objects:'supportedBy',test:'Shelffloor2')),
	tell(triple(test:'Tabletop1', hsr_objects:'sourceOrTarget',source)),
	tell(triple(test:'Shelffloor1', hsr_objects:'sourceOrTarget',source)),
	tell(triple(test:'Shelffloor2', hsr_objects:'sourceOrTarget',target)),
	tell(triple(test:'BucketOpening1', hsr_objects:'sourceOrTarget',target)).
	

test(surface_type_of) :-
	triple(Surface,hsr_objects:'isSurfaceType',Expected),
	surface_type_of(Surface,Actual),
	assert_equals(Actual,Expected).

test(is_surface_when_input_is_surface) :-
	triple(Surface,hsr_objects:'isSurfaceType',table),
	is_surface(Surface).

test(is_surface_when_input_is_object, fail) :-
	has_type(ObjectInstance,hsr_objects:'CoffeeMug'),
	is_surface(ObjectInstance).

test(is_table_when_input_is_tabletop) :-
	triple(Tabletop,hsr_objects:'isSurfaceType',table),
	is_table(Tabletop).

test(is_table_when_input_is_object, fail) :-
	has_type(ObjectInstance,hsr_objects:'CoffeeMug'),
	is_table(ObjectInstance).

test(is_bucket_when_input_is_bucketopening) :-
	triple(BucketOpening,hsr_objects:'isSurfaceType',bucket),
	is_bucket(BucketOpening).

test(is_bucket_when_input_is_object, fail) :-
	has_type(ObjectInstance,hsr_objects:'CoffeeMug'),
	is_bucket(ObjectInstance).

test(is_shelf_when_input_is_shelffloor) :-
	triple(Shelffloor,hsr_objects:'isSurfaceType', shelf),
	is_shelf(Shelffloor).

test(is_shelf_when_input_is_object, fail) :-
	has_type(ObjectInstance,hsr_objects:'CoffeeMug'),
	is_shelf(ObjectInstance).

test(all_surfaces) :-
	all_surfaces(Surfaces),
	assert_true(Surfaces == [test:'Shelffloor1',test:'Shelffloor2',test:'Tabletop1',test:'Tabletop2',test:'BucketOpening1',test:'BucketOpening2']).

test(shelf_surfaces) :-
	shelf_surfaces(Shelffloors),
	assert_true(Shelffloors == [test:'Shelffloor1', test:'Shelffloor2']).

test(table_surfaces) :-
	table_surfaces(Tabletops),
	assert_true(Tabletops == [test:'Tabletop1', test:'Tabletop2']).

test(bucket_surfaces) :-
	bucket_surfaces(Bucketopenings),
	assert_true(Bucketopenings == [test:'BucketOpening1', test:'BucketOpening2']).

test(find_supporting_surface) :-
	triple(ObjectInstance,hsr_objects:'supportedBy',test:'Tabletop1'),
	find_supporting_surface(ObjectInstance, Surface),
	assert_true(Surface == test:'Tabletop1').

test(objects_on_surface) :-
	triple(test:'Chips1',hsr_objects:'supportedBy',Surface),
	objects_on_surface(Objects,Surface),
	assert_true(Objects == [test:'Chips1', test:'IceTea1']).

test(objects_on_list_of_surfaces) :-
	findall(Surface, has_type(Surface,hsr_objects:'Tabletop'),SurfaceList),
	objects_on_list_of_surfaces(ObjectInstances,SurfaceList),
	assert_true(ObjectInstances == [test:'Chips1',test:'IceTea1']).

test(all_objects_on_source_surfaces) :-
	all_objects_on_source_surfaces(ObjectInstances),
	assert_true(ObjectInstances == [test:'Chips1', test:'IceTea1']).

test(all_objects_on_target_surfaces) :-
	all_objects_on_target_surfaces(ObjectInstances),
	assert_true(ObjectInstances == [test:'CoffeeMug1']).

test(all_objects_on_tables) :-
	all_objects_on_tables_(ObjectInstances),
	assert_true(ObjectInstances == [test:'Chips1', test:'IceTea1']).

test(all_objects_in_buckets) :-
	all_objects_in_buckets(ObjectInstances),
	assert_true(ObjectInstances == []).
	
test(assert_object_on) :-
	has_type(ObjectInstance,hsr_objects:'IceTea'),
	has_type(Surface,hsr_objects:'BucketOpening'),
	assert_object_on(ObjectInstance,Surface),
	assert_false(triple(test:'IceTea1',hsr_objects:'supportedBy',test:'Tabletop1')),
	assert_true(triple(test:'IceTea1',hsr_objects:'supportedBy',test:'BucketOpening1')).

test(get_surface_role) :-
	triple(Source,hsr_objects:'sourceOrTarget',source),
	get_surface_role(Source,RoleS),
	assert_true(RoleS == source),
	triple(Target,hsr_objects:'sourceOrTarget',target),
	get_surface_role(Target, RoleT),
	assert_true(RoleT == target).

test(make_all_surface_type_role, setup(cleanup_roles)) :-
	make_all_surface_type_role(bucket,source),
	findall(Surface,triple(Surface,hsr_objects:'sourceOrTarget',source),Sources),
	assert_true(Sources == [test:'BucketOpening1',test:'BucketOpening2']).

test(make_surfaces_source, setup(cleanup_roles)) :-
	findall(Surface, has_type(Surface,hsr_objects:'Tabletop'),Tables),	
	findall(Surface, has_type(Surface,hsr_objects:'BucketOpening'),Buckets),
	append(Tables,Buckets,SurfaceList),
	make_surfaces_source(SurfaceList),
	findall(Surface,triple(Surface,hsr_objects:'sourceOrTarget',source),Sources),
	assert_true(Sources == [test:'Tabletop1',test:'Tabletop2',test:'BucketOpening1',test:'BucketOpening2']).

test(make_surfaces_target,setup(cleanup_roles)) :-
	findall(Surface, has_type(Surface,hsr_objects:'Tabletop'),Tables),	
	findall(Surface, has_type(Surface,hsr_objects:'BucketOpening'),Buckets),
	append(Tables,Buckets,SurfaceList),
	make_surfaces_target(SurfaceList),
	findall(Surface,triple(Surface,hsr_objects:'sourceOrTarget',target),Targets),
	assert_true(Targets == [test:'Tabletop1',test:'Tabletop2',test:'BucketOpening1',test:'BucketOpening2']).

test(make_role, setup(cleanup_roles)) :-
	has_type(Source,hsr_objects:'Shelffloor'),
	make_role(Source,source),
	assert_true(triple(Source,hsr_objects:'sourceOrTarget',source)),
	has_type(Target,hsr_objects:'Tabletop'),
	make_role(Target,target),
	assert_true(triple(Target,hsr_objects:'sourceOrTarget',target)).


test(fail) :-
    fail.

:- end_tripledb_tests('surfaces').
