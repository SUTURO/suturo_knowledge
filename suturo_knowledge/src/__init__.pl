% You can register other knowrob packages here. 
:- register_ros_package(knowrob).
:- register_ros_package(suturo_knowledge).

:- load_owl('package://suturo_knowledge/owl/suturo.owl',
	    % Each option can only be used once
	    [namespace(suturo,   'http://www.ease-crc.org/ont/SUTURO#')]).

% Import rdf_register_ns/3 into here so it can be used to register additional namespaces
:- use_module(library('semweb/rdf_db'), 
		[ rdf_register_ns/3 ]).

% Register soma Namespaces here, as we can only use one option in the load_owl predicate.
:- rdf_register_ns(soma,      'http://www.ease-crc.org/ont/SOMA.owl#',      [keep(true)]).
:- rdf_register_ns(soma_home, 'http://www.ease-crc.org/ont/SOMA-HOME.owl#', [keep(true)]).

:- use_directory('model').

:- ros_param_get_string("/suturo_room_viz/urdf_param", Param),
   load_urdf_from_param(Param).

:- tf_logger_enable.

:- init_furnitures.
