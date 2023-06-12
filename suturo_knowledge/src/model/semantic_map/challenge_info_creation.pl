%% This module initializes challenge specific predefined information.
:- module(challenge_info_creation,
	  [
            init_serving_breakfast/0,
            init_storing_groceries/0
	  ]).

:- use_module(library('util/util'),
      [
      ros_info/2
      ]).

%% init_serving_breakfast is det.
%
%  Initializes the object info for serving breakfast.
%
init_serving_breakfast :-
    ros_info('Initializing object info for serving breakfast...', []),
    has_urdf_name(OriginLocation, 'shelf:shelf:shelf_base_center'),
    kb_project(holds(soma:'Bowl', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'CerealBox', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'Spoon', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'MilkPack', suturo:hasOriginLocation, OriginLocation)),
    has_urdf_name(DestinationLocation, 'left_table:table:table_front_edge_center'),
    kb_project(holds(soma:'Bowl', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'CerealBox', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'Spoon', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'MilkPack', suturo:hasDestinationLocation, DestinationLocation)),
    ros_info('Serving breakfast initialized.', []).

%% init_storing_groceries is det.
%
%  Initializes the object info for storing groceries.
%
init_storing_groceries :-
    ros_info('Initializing object info for storing groceries...', []),
    foreach((member(X,[2,1]),
             atom_concat('shelf:shelf:shelf_floor_', X, UrdfName),
             has_urdf_name(DestinationLocation, UrdfName)),
            % TODO fix expanding namespaces to use namespace:Resource here.
            kb_project(triple('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject',
                              'http://www.ease-crc.org/ont/SUTURO.owl#hasDestinationLocation',
                              DestinationLocation))),
    ros_info('Storing groceries initialized.', []).
