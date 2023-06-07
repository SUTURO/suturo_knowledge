%% This module initializes predefined information about objects in the environment
:- module(object_info_creation,
	  [
        init_object_info_serving_breakfast/0
	  ]).

:- use_module(library('util/util'),
      [
      ros_info/2
      ]).

init_object_info_serving_breakfast :-
    ros_info('Initializing object info for serving breakfast', []),
    has_urdf_name(OriginLocation, 'right_table:table:table_front_edge_center'),
    kb_project(holds(soma:'Bowl', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'CerealBox', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'Spoon', suturo:hasOriginLocation, OriginLocation)),
    kb_project(holds(soma:'MilkPack', suturo:hasOriginLocation, OriginLocation)),
    has_urdf_name(DestinationLocation, 'left_table:table:table_front_edge_center'),
    kb_project(holds(soma:'Bowl', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'CerealBox', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'Spoon', suturo:hasDestinationLocation, DestinationLocation)),
    kb_project(holds(soma:'MilkPack', suturo:hasDestinationLocation, DestinationLocation)).