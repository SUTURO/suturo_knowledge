:- register_ros_package(knowrob_common).
:- register_ros_package(knowrob_objects).
:- register_ros_package(knowrob_actions).
:- register_ros_package(knowledge).
:- register_ros_package(knowrob_vis).
:- register_ros_package(knowrob_memory).
:- register_ros_package(rosprolog).
:- register_ros_package(urdfprolog).

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

% :- use_module(library('rdf_urdf')).
% :- use_module(library('urdf_parser')).

:- use_module(library('config')).
:- use_module(library('pickup')).
:- use_module(library('object_state')).
:- use_module(library('surfaces')).
:- use_module(library('beliefstate')).
:- use_module(library('spatial_comp')).
:- use_module(library('assignplaces')).



:- owl_parser:owl_parse('package://knowrob_common/owl/knowrob.owl').
:- owl_parser:owl_parse('package://knowledge/owl/objects.owl').
:- owl_parser:owl_parse('package://urdfprolog/owl/urdf.owl').

:- ros_package_path('knowledge', X), % TODO Change to load form param
    atom_concat(X, '/urdf/hsrb_lab.urdf', FileURL),
    kb_create(urdf:'Robot', Robot),
    rdf_urdf_load_file(Robot, FileURL).


:- forall(supporting_surface(SurfaceLink), assert_surface_types(SurfaceLink)).
