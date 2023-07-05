%% This module initializes challenge specific predefined information.
:- module(challenge_info_creation,
	  [
            init_serve_breakfast/0,
            init_storing_groceries/0,
            init_clean_the_table/0,
	        init_clean_the_table_no_dishwasher/0,
            init_gpsr/0,
            init_predefined_names_robocup_2023/0
	  ]).

:- rdf_meta(activate_challenge(r)).
:- rdf_meta(activate_challenge(r, -)).
:- rdf_meta(log_set(r,r,r)).

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
             atom_concat('open_shelf:shelf:shelf_floor_', X, UrdfName),
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

init_clean_the_table_no_dishwasher :-
      ros_info('Initializing info for "Clean the Table"...', []),
      activate_challenge(suturo:'CleanTheTable'),
      has_urdf_name(OriginLocation, 'left_table:table:table_front_edge_center'),
      kb_project(holds(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation)),
      has_urdf_name(DestinationLocation, 'kitchen_table:table:table_front_edge_center'),
      kb_project(holds(dul:'PhysicalObject', suturo:hasDestinationLocation, DestinationLocation)),
      ros_info('"Clean the Table" initialized.', []).


%% init_gpsr is det.
%
% Initializes the predefined info for general purpouse service robot.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of the challenge.
%
init_gpsr :-
      ros_info('Initializing info for "gpsr"...'),
      activate_challenge(suturo:'GPSR'),
      forall(
          (
              is_kitchen(Kitchen),
              is_inside_of(Furniture,Kitchen),
              once((is_table(Furniture);is_shelf(Furniture)))
          ),
          log_set(soma:'Cutlery', suturo:hasOriginLocation, Furniture)),
      forall(
          (
              is_kitchen(Kitchen),
              is_inside_of(Furniture,Kitchen),
              is_shelf(Furniture)
          ),
          log_set(suturo:'Fruit', suturo:hasOriginLocation, Furniture)),
      forall(
          (
              is_living_room(LivingRoom),
              is_inside_of(Furniture,LivingRoom),
              is_table(Furniture)
          ),
          log_set(soma:'Cup', suturo:hasOriginLocation, Furniture)),
      forall(
          (
              is_living_room(LivingRoom),
              is_inside_of(Furniture,LivingRoom),
              is_shelf(Furniture)
          ),
          log_set(suturo:'CrackerBox', suturo:hasOriginLocation, Furniture)),
      ros_info('"GPSR" initialized.').

log_set(S,P,O) :-
    kb_project(triple(S,P,O)),
    ros_info('Set ~w as ~w for ~w', [O, P, S]).

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

%% init_predefined_names_robocup_2023 is det.
%
% Initializes the predefined names of RoboCup 2023 Bordeaux for the challenges.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of knowledge.
%
init_predefined_names_robocup_2023 :-
    kb_project(holds(suturo:'AbrasiveSponge', suturo:hasPredefinedName, 'sponge')),
    kb_project(holds(suturo:'BleachCleanserBottle', suturo:hasPredefinedName, 'cleanser')),
    kb_project(holds(soma:'WineBottle', suturo:hasPredefinedName, 'red wine')),
    kb_project(holds(suturo:'JuicePack', suturo:hasPredefinedName, 'juice pack')),
    kb_project(holds(suturo:'ColaCan', suturo:hasPredefinedName, 'cola')),
    kb_project(holds(suturo:'TropicalJuiceBottle', suturo:hasPredefinedName, 'tropical juice')),
    kb_project(holds(soma:'MilkBottle', suturo:hasPredefinedName, 'milk')),
    kb_project(holds(suturo:'IceTeaBottle', suturo:hasPredefinedName, 'iced tea')),
    kb_project(holds(suturo:'OrangeJuiceBox', suturo:hasPredefinedName, 'orange juice')),
    kb_project(holds(suturo:'TunaFishCan', suturo:hasPredefinedName, 'tuna')),
    kb_project(holds(suturo:'TomatoSoupCan', suturo:hasPredefinedName, 'tomato soup')),
    kb_project(holds(suturo:'PottedMeatCan', suturo:hasPredefinedName, 'spam')),
    kb_project(holds(suturo:'MustardBottle', suturo:hasPredefinedName, 'mustard')),
    kb_project(holds(suturo:'JellOStrawberryBox', suturo:hasPredefinedName, 'strawberry jello')),
    kb_project(holds(suturo:'JellOChocolatePuddingBox', suturo:hasPredefinedName, 'chocolate jello')),
    kb_project(holds(suturo:'CoffeeCan', suturo:hasPredefinedName, 'coffee grounds')),
    kb_project(holds(suturo:'Pear', suturo:hasPredefinedName, 'pear')),
    kb_project(holds(suturo:'Plum', suturo:hasPredefinedName, 'plum')),
    kb_project(holds(suturo:'Peach', suturo:hasPredefinedName, 'peach')),
    kb_project(holds(suturo:'Lemon', suturo:hasPredefinedName, 'lemon')),
    kb_project(holds(suturo:'Orange', suturo:hasPredefinedName, 'orange')),
    kb_project(holds(suturo:'Banana', suturo:hasPredefinedName, 'banana')),
    kb_project(holds(suturo:'Strawberry', suturo:hasPredefinedName, 'strawberry')),
    kb_project(holds(suturo:'Apple', suturo:hasPredefinedName, 'apple')),
    kb_project(holds(suturo:'TennisBall', suturo:hasPredefinedName, 'tennis ball')),
    kb_project(holds(suturo:'MiniSoccerBall', suturo:hasPredefinedName, 'soccer ball')),
    kb_project(holds(suturo:'RubiksCube'	, suturo:hasPredefinedName, 'rubiks cube')),
    kb_project(holds(suturo:'Dice', suturo:hasPredefinedName, 'dice')),
    kb_project(holds(suturo:'Baseball', suturo:hasPredefinedName, 'baseball')),
    kb_project(holds(suturo:'PringlesChipsCan'	, suturo:hasPredefinedName, 'pringles')),
    kb_project(holds(suturo:'Strawberry', suturo:hasPredefinedName, 'strawberry')),
    kb_project(holds(suturo:'CerealBoxRoboCup', suturo:hasPredefinedName, 'cornflakes')),
    kb_project(holds(suturo:'CrackerBox', suturo:hasPredefinedName, 'cheezit')),
    kb_project(holds(soma:'Spoon', suturo:hasPredefinedName, 'spoon')),
    kb_project(holds(soma:'Plate', suturo:hasPredefinedName, 'plate')),
    kb_project(holds(soma:'Cup', suturo:hasPredefinedName, 'cup')),
    kb_project(holds(soma:'Fork', suturo:hasPredefinedName, 'fork')),
    kb_project(holds(soma:'Bowl', suturo:hasPredefinedName, 'bowl')),
    kb_project(holds(soma:'Knife', suturo:hasPredefinedName, 'knife')),
    ros_info('RoboCup 2023 predefined object names initialized').