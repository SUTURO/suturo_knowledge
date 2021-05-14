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

:- use_module(library('config')).
:- use_module(library('urdf')).
:- use_module(library('spatial_comp')).
:- use_module(library('pickup')).
:- use_module(library('object_state')).
:- use_module(library('surfaces')).
:- use_module(library('beliefstate')).
:- use_module(library('assignplaces')).
:- use_module(library('gripper'), [gripper/1, gripper_init/1]).
:- use_module(library('rooms')).
:- use_module(library('doors')).
:- use_module(library('export')).
:- use_module(library('nlg')).
:- use_module(library('algebra')).
:- use_module(library('algorithms')).

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
:- tripledb_load(
	'package://knowledge/owl/locations.owl',
	[ namespace(hsr_locations, 'http://www.semanticweb.org/suturo/ontologies/2021/0/locations#')
	]).
:- tripledb_load(
	'http://knowrob.org/kb/URDF.owl',
	[ namespace(urdf, 'http://knowrob.org/kb/urdf.owl#')
	]).

:- ros_param_get_string('/param_to_load_URDF_from', Param),
    load_surfaces_from_param(Param).

:- tf_lookup_transform(map, map, _).

:- gripper(Gripper), gripper_init(Gripper).

:- init_rooms.
:- init_doors.
:- init_door_paths.
:- init_furnitures.

