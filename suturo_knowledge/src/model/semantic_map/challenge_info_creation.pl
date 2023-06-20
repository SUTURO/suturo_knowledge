%% This module initializes challenge specific predefined information.
:- module(challenge_info_creation,
	  [
            init_serve_breakfast/0,
            init_storing_groceries/0,
            init_clean_the_table/0
	  ]).

:- rdf_meta(activate_challenge(r)).
:- rdf_meta(activate_challenge(r, -)).

:- use_module(library('util/util'),
      [
      ros_info/2
      ]).

%% init_serve_breakfast is det.
%
% Initializes the predefined info for serving breakfast.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of the challenge.
%
init_serve_breakfast :-
    ros_info('Initializing info for "Serving Breakfast"...', []),
    activate_challenge(suturo:'ServeBreakfast'),
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
    ros_info('"Serving Breakfast" initialized.', []).

%% init_storing_groceries is det.
%
% Initializes the predefined info for storing groceries.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of the challenge.
%
init_storing_groceries :-
      ros_info('Initializing info for "Storing Groceries"...', []),
      activate_challenge(suturo:'StoringGroceries'),
      has_urdf_name(OriginLocation, 'left_table:table:table_front_edge_center'),
      kb_project(holds(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation)),
      foreach((member(X,[2,1]),
             atom_concat('shelf:shelf:shelf_floor_', X, UrdfName),
             has_urdf_name(DestinationLocation, UrdfName)),
            % TODO fix expanding namespaces to use namespace:Resource here.
            kb_project(holds('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject',
                             'http://www.ease-crc.org/ont/SUTURO.owl#hasDestinationLocation',
                              DestinationLocation))),
      ros_info('"Storing Groceries" initialized.', []).

%% init_clean_the_table is det.
%
% Initializes the predefined info for clean the table.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of the challenge.
%
init_clean_the_table :-
      ros_info('Initializing info for "Clean the Table"...', []),
      activate_challenge(suturo:'CleanTheTable'),
      has_urdf_name(OriginLocation, 'left_table:table:table_front_edge_center'),
      kb_project(holds(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation)),
      has_urdf_name(DestinationLocation, 'imaginary_dishwasher:dishwasher_tray_bottom'),
      kb_project(holds(dul:'PhysicalObject', suturo:hasDestinationLocation, DestinationLocation)),
      ros_info('"Clean the Table" initialized.', []).

%% activate_challenge(+Challenge) is det.
%
% Activates the given challenge.
%
% @param Challenge The challenge class to activate.
% @param ActivatedChallenge The activated challenge object.
%
activate_challenge(Challenge) :-
      activate_challenge(Challenge, _).
activate_challenge(Challenge, ActivatedChallenge) :-
      from_current_scope(Scope),
      kb_project(is_type(ActivatedChallenge, Challenge), Scope).