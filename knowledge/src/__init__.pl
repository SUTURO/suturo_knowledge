:- register_ros_package(rosprolog).
:- register_ros_package(knowrob).
:- register_ros_package(knowledge).

%%% KnowRob imports
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).
:- use_module(library('db/tripledb'), [tripledb_load/1, tripledb_load/2, tripledb_tell/5, tripledb_forget/3]).
:- use_module(library('utility/url'),[ros_package_iri/2]).
:- use_module(library('lang/terms/triple')).
:- use_module(library('model/metrics/WuPalmer')).
:- use_module(library('lang/terms/is_at'), [is_at/2]).
:- use_module(library('utility/algebra'), [transform_close_to/3]).

:- use_module(library('gripper/gripper_info'), 
	[
		init_gripper/0,
		all_objects_in_gripper/1
	]).
:- use_module(library('gripper/grasping'),
	[
		surface_pose_to_perceive_from/2,
		object_pose_to_grasp_from/2,
		attach_object_to_gripper/1
	]).
:- use_module(library('gripper/placing'), 
	[
		object_goal_pose_offset/3, 
		release_object_from_gripper/1
	]).
:- use_module(library('gripper/open_door')).
:- use_module(library('beliefstate'), 
	[
		object_goal_surface/2, 
		group_objects_at/1
	]).
:- use_module(library('model/environment/furnitures'), [init_furnitures/0]).
:- use_module(library('model/objects/object_creation'), [create_object/9]).
:- use_module(library('model/objects/object_manipulation'), [set_object_handeled/1]).
:- use_module(library('config')).
:- use_module(library('urdf')).
:- use_module(library('model/environment/surfaces'), 
	[
		set_surface_visited/1,
		set_surface_not_visited/1,
		surfaces_not_visited/1,
		get_perception_surface_region/2
	]).
:- use_module(library('model/environment/rooms'), [init_rooms/0, connect_rooms/0, all_rooms/1]).
:- use_module(library('model/environment/doors'), 
	[	
		init_doors/0, 
		init_door_paths/0,
		get_door_state/2
	]).
:- use_module(library('locations/actual_locations'), 
	[
		is_legal_obj_position/1,
		pose_is_outside/1, 
		surfaces_not_visited_in_room/2, 
		robot_in_room/1, 
		surfaces_in_room/2,
		object_supported_by_surface/2
	]).
:- use_module(library('naturallanguage/nlg')).
:- use_module(library('naturallanguage/nlp')).
:- use_module(library('applications/next_object'), [next_object/2]).
:- use_module(library('applications/path_finder'), [shortest_path_between_rooms/3]).
:- use_module(library('applications/clean_table'), [temporary_storage_surface/1, temporary_storage_pose/2]).
:- use_module(library('locations/spatial_comp'),
	[
		hsr_lookup_transform/4,
		surface_front_edge_center_pose/2,
		surface_center_pose/2,
		surface_dimensions/4,
		object_pose/2
	]).
:- ros_package_iri(knowledge, 'package://knowledge/owl/objects.owl').
:- ros_package_iri(knowledge, 'package://knowledge/owl/rooms.owl').
:- ros_package_iri(knowledge, 'package://knowledge/owl/locations.owl').


:- tripledb_load(
	'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl',
	[ namespace(dul)
	]).
:- tripledb_load(
	'http://knowrob.org/kb/knowrob.owl',
	[ namespace(knowrob)
	]).
:- tripledb_load('package://knowledge/owl/objects.owl',
	[ namespace(hsr_objects)
	]).
:- tripledb_load(
	'package://knowledge/owl/rooms.owl',
	[ namespace(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#')
	]).

:- ros_param_get_string('/locations_ontology_file', OntologyFileName), 
	string_concat('package://knowledge/owl/locations/', OntologyFileName, PathToOntology),
	tripledb_load(
		PathToOntology,
		[ namespace(hsr_locations)
		]).

%:- tripledb_load(
%		'package://knowledge/owl/locations.owl',
%		[ namespace(hsr_locations, 'http://www.semanticweb.org/suturo/ontologies/2021/0/locations#'),
%	  	graph(locations)
%		]).

:- tripledb_load(
	'http://knowrob.org/kb/URDF.owl',
	[ namespace(urdf, 'http://knowrob.org/kb/urdf.owl#')
	]).

:- ros_param_get_string('/param_to_load_URDF_from', Param),
    load_surfaces_from_param(Param).

:- tf_logger_enable.

:- ignore(hsr_lookup_transform('map', 'base_footprint', _, _)). % Why does this help with TF erros??

:- init_gripper.

:- init_rooms.
:- init_doors.
:- connect_rooms.
:- init_door_paths.
:- init_furnitures.

