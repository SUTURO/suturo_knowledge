% You can register other knowrob packages here.
:- register_ros_package(knowrob).
:- register_ros_package(suturo_knowledge).

% Load the main SUTURO ontology
:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).

% Imports to register additional namespaces
:- use_module(library('semweb/rdf_db'),
		[ rdf_register_prefix/3 ]).

% Register other namespaces here, as we can only use one option in the load_owl predicate.
:- rdf_register_prefix(soma, 'http://www.ease-crc.org/ont/SOMA.owl#', [keep(true)]).
:- rdf_register_prefix(soma_home, 'http://www.ease-crc.org/ont/SOMA-HOME.owl#', [keep(true)]).
:- rdf_register_prefix(soma_obj, 'http://www.ease-crc.org/ont/SOMA-OBJ.owl#', [keep(true)]).
:- rdf_register_prefix(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_register_prefix(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).

%% Make sure utils are loaded before model and other directories
:- use_directory('util').

%% Load the object_shape workaround
:- ensure_loaded('shape_workaround').

:- use_directory('model').
:- use_directory('reasoning').

:- once((  ros_param_get_string("/suturo_room_viz/urdf_param", Param),
           load_urdf_from_param(Param)
        ;  ros_warn('No semantic map loaded!'))).

:- tf_logger_enable.

% init the rooms before the furniture to make sure the furniture
% is assigned its room.
:- init_rooms.
:- init_semantic_map.
