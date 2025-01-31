%% This module initializes challenge specific predefined information.
:- module(challenge_info_creation,
	  [
            init_serve_breakfast/0,
            init_storing_groceries/0,
            init_clean_the_table/0,
            init_gpsr/0,
            init_locations_robocup_2023/0,
			init_gpsr_2024/0
            %init_predefined_names_robocup_2023/0
	  ]).

:- rdf_meta(activate_challenge(r)).
:- rdf_meta(activate_challenge(r, -)).
:- rdf_meta(log_set(r,r,r)).

:- use_module(library('util/util'),
      [
      ros_info/2
      ]).

:- use_module(library('model/object/artifact/furniture/furniture_info'),
              [ has_robocup_name/2 ]).

:- use_module(library('model/urdf'), [ has_urdf_name/2 ]).

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
    once((has_urdf_name(OriginLocation, 'pantry:shelf:shelf_floor_1'), %TODO should be layer but then also in semantic map
           log_set(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation))
          ;
         ((is_shelf(Shelf),
          object_pose(Shelf,[map,[_,_,Z],_Rot]),
          Z<0.8,
          Z>0.6),
          log_set(dul:'PhysicalObject', suturo:hasOriginLocation, Shelf))),
    once((has_urdf_name(OriginLocation, 'cupboard:cupboard:table_center'),
           log_set(dul:'PhysicalObject', suturo:hasDestinationLocation, OriginLocation))
          ;
         (is_table(Table),
          log_set(dul:'PhysicalObject', suturo:hasDestinationLocation, Table))),
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
      once((has_robocup_name(OriginLocation, storing_groceries_table),
            log_set(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation))
          ;
          (is_table(Table),
           log_set(dul:'PhysicalObject', suturo:hasOriginLocation, Table))),
      (  has_robocup_name(_,pantry)
      -> forall((has_robocup_name(DestinationLocation,pantry),
                 is_shelf(DestinationLocation),
                 object_pose(DestinationLocation,[map,[_,_,Z],_]),
                 Z<1.1),
                log_set(dul:'PhysicalObject',
                        suturo:'hasDestinationLocation',
                        DestinationLocation))
      ;  forall((is_shelf(DestinationLocation),
                 object_pose(DestinationLocation,[map,[_,_,Z],_]),
                 Z<1.1),
                log_set(dul:'PhysicalObject',
                        suturo:'hasDestinationLocation',
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
      once((has_urdf_name(OriginLocation, 'kitchen_table:kitchen_table:table_center'),
       log_set(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation))
      ;
       (is_table(Table),
        log_set(dul:'PhysicalObject', suturo:hasOriginLocation, Table))),
      once((has_urdf_name(OriginLocation2, 'dishwasher:dishwasher:dishwasher_tray_2_bottom'),
       log_set(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation2))
      ;
       (is_shelf(DestinationLocation),
        log_set(dul:'PhysicalObject', suturo:hasOriginLocation, DestinationLocation))),
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
      init_locations_robocup_2023,
      ros_info('"GPSR" initialized.').

log_set(S,P,O) :-
    kb_project(holds(S,P,O)),
    iri_xml_namespace(S,_,WS),
    iri_xml_namespace(P,_,WP),
    iri_xml_namespace(O,_,WO),
    ros_info('Set ~w ~w ~w', [WS, WP, WO]).

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

%% init_locations_robocup_2023 is det.
%
% Initializes the predefined locations for RoboCup@Home 2023.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of the challenge.
%
init_locations_robocup_2023 :-
    forall(
        (
          is_bedroom(BedRoom),
          is_inside_of(Furniture, BedRoom),
          is_shelf(Furniture)
        ),
        log_set(suturo:'RoboCupCleaningSupplies', suturo:hasOriginLocation, Furniture)),
    has_urdf_name(Cabinet, 'cabinet:cabinet:shelf_floor_1'), %TODO: should be layer
    log_set(suturo:'RoboCupDrinks', suturo:hasOriginLocation, Cabinet),
    forall((has_robocup_name(PantryShelf, pantry),
            is_shelf(PantryShelf)),
               log_set(suturo:'RoboCupFood', suturo:hasOriginLocation, PantryShelf)
           ),
    has_urdf_name(Desk, 'desk:desk:table_center'),
    log_set(suturo:'RoboCupFruits', suturo:hasOriginLocation, Desk),
    forall(member(X,[0,1,2,3]),
           (
               atom_concat('bookshelf:bookshelf:shelf_floor_',X,LayerName),
               has_urdf_name(Layer,LayerName),
               log_set(suturo:'RoboCupToys', suturo:hasOriginLocation, Layer)
           )),
    forall(member(X,[1,2]),
           (
               atomic_list_concat(['side_table',X,':side_table:table_center'], TableName),
               has_urdf_name(Table, TableName),
               log_set(suturo:'RoboCupSnacks', suturo:hasOriginLocation, Table)
           )),
    forall(is_kitchen_table(Table),
           (
               log_set(suturo:'RoboCupDishes', suturo:hasOriginLocation, Table)
           )),
    ros_info('Set origin locations according to robocup 2023 data').


%% init_predefined_names_robocup_2023 is det.
%
% Initializes the predefined names of RoboCup 2023 Bordeaux for the challenges.
%
% This is a temporary solution until the challenge info can be loaded from an ontology.
% This predicate should only be called once at the start of knowledge.

init_predefined_names_robocup_2023 :-
    kb_project((holds(suturo:'RoboCupCleaningSupplies', suturo:hasPredefinedName, 'cleaning supplies'),
                holds(suturo:'AbrasiveSponge', suturo:hasPredefinedName, 'sponge'),
                holds(suturo:'BleachCleanserBottle', suturo:hasPredefinedName, 'cleanser'),
                holds(suturo:'RoboCupDrinks', suturo:hasPredefinedName, 'drinks'),
                holds(soma:'WineBottle', suturo:hasPredefinedName, 'red wine'),
                holds(suturo:'JuicePack', suturo:hasPredefinedName, 'juice pack'),
                holds(suturo:'ColaCan', suturo:hasPredefinedName, 'cola'),
                holds(suturo:'TropicalJuiceBottle', suturo:hasPredefinedName, 'tropical juice'),
                holds(soma:'MilkBottle', suturo:hasPredefinedName, 'milk'),
                holds(suturo:'IceTeaBottle', suturo:hasPredefinedName, 'iced tea'),
                holds(suturo:'OrangeJuiceBox', suturo:hasPredefinedName, 'orange juice'),
                holds(suturo:'RoboCupFood', suturo:hasPredefinedName, 'food'),
                holds(suturo:'TunaFishCan', suturo:hasPredefinedName, 'tuna'),
                holds(suturo:'TomatoSoupCan', suturo:hasPredefinedName, 'tomato soup'),
                holds(suturo:'PottedMeatCan', suturo:hasPredefinedName, 'spam'),
                holds(suturo:'MustardBottle', suturo:hasPredefinedName, 'mustard'),
                holds(suturo:'JellOStrawberryBox', suturo:hasPredefinedName, 'strawberry jello'),
                holds(suturo:'JellOChocolatePuddingBox', suturo:hasPredefinedName, 'chocolate jello'),
                holds(suturo:'CoffeeCan', suturo:hasPredefinedName, 'coffee grounds'),
                holds(suturo:'RoboCupFruits', suturo:hasPredefinedName, 'fruits'),
                holds(suturo:'Pear', suturo:hasPredefinedName, 'pear'),
                holds(suturo:'Plum', suturo:hasPredefinedName, 'plum'),
                holds(suturo:'Peach', suturo:hasPredefinedName, 'peach'),
                holds(suturo:'Lemon', suturo:hasPredefinedName, 'lemon'),
                holds(suturo:'Orange', suturo:hasPredefinedName, 'orange'),
                holds(suturo:'Banana', suturo:hasPredefinedName, 'banana'),
                holds(suturo:'Strawberry', suturo:hasPredefinedName, 'strawberry'),
                holds(suturo:'Apple', suturo:hasPredefinedName, 'apple'),
                holds(suturo:'RoboCupToys', suturo:hasPredefinedName, 'toys'),
                holds(suturo:'TennisBall', suturo:hasPredefinedName, 'tennis ball'),
                holds(suturo:'MiniSoccerBall', suturo:hasPredefinedName, 'soccer ball'),
                holds(suturo:'RubiksCube', suturo:hasPredefinedName, 'rubiks cube'),
                holds(suturo:'Dice', suturo:hasPredefinedName, 'dice'),
                holds(suturo:'Baseball', suturo:hasPredefinedName, 'baseball'),
                holds(suturo:'RoboCupSnacks', suturo:hasPredefinedName, 'snacks'),
                holds(suturo:'PringlesChipsCan', suturo:hasPredefinedName, 'pringles'),
                holds(suturo:'Strawberry', suturo:hasPredefinedName, 'strawberry'),
                holds(suturo:'CerealBoxRoboCup', suturo:hasPredefinedName, 'cornflakes'),
                holds(suturo:'CrackerBox', suturo:hasPredefinedName, 'cheezit'),
                holds(suturo:'RoboCupDishes', suturo:hasPredefinedName, 'dishes'),
                holds(soma:'Spoon', suturo:hasPredefinedName, 'spoon'),
                holds(soma:'Plate', suturo:hasPredefinedName, 'plate'),
                holds(soma:'Cup', suturo:hasPredefinedName, 'cup'),
                holds(soma:'Fork', suturo:hasPredefinedName, 'fork'),
                holds(soma:'Bowl', suturo:hasPredefinedName, 'bowl'),
                holds(soma:'Knife', suturo:hasPredefinedName, 'knife'))),
    ros_info('RoboCup 2023 predefined object names initialized').

init_gpsr_2024 :-
	ros_info('Initializing origin and destination locations for gpsr 2024'),

    has_type(Office, 'http://www.ease-crc.org/ont/SUTURO.owl#Office'),

    % decorations
    ros_warn('TODO: Use decorations class'),
    is_inside_of(Desk, Office),
    is_table(Desk),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDecorations', Desk),

    % cleaning supplies are on the shelf in the Office
    forall((
                  is_inside_of(ShelfLayer, Office),
                  is_shelf_layer(ShelfLayer)
           ),
           log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupCleaningSupplies', ShelfLayer)
          ),

    % toys on tv table in living room
    has_robocup_name(TvTable, tv_table),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupToys', TvTable),

    % fruits on the coffe table (round table in living room)
    has_robocup_name(CoffeTable, coffee_table),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFruits', CoffeTable),

    % drinks in kitchen cabinet in kitchen
    is_kitchen(Kitchen),
    forall((
                  is_inside_of(ShelfLayer, Kitchen),
                  is_shelf_layer(ShelfLayer)
           ),
           log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupCleaningSupplies', ShelfLayer)
          ),

    % snacks on dinner table in kitchen
    has_robocup_name(DiningTable, dinner_table),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupSnacks', DiningTable),

    % dishes in dishwasher in kitchen
    ros_warn('TODO: Check if dishwasher is loaded into knowledge database'),
    %% has_type(DishwasherTray, suturo:'DishwasherTray'),
    %% log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDishes', DishwasherTray),
    has_robocup_name(DishwasherTable, d_table),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDishes', DishwasherTable),

    % food on kitchen counter in kitchen
    has_robocup_name(KitchenCounter, kitchen_counter),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupFood', KitchenCounter),

     % drinks on kitchen cabinet
    has_robocup_name(KitchenCabinet, kitchen_cabinet),
    log_set_both('http://www.ease-crc.org/ont/SUTURO.owl#RoboCupDrinks', KitchenCabinet),


    %% Altes zeug
	%% % geschirr wie plate, cup, cutlery auf popcorn table
	%% has_urdf_name(PopcornTable, 'popcorn_table:p_table:table_center'),
	%% log_set(suturo:'RoboCupDishes', suturo:hasOriginLocation, PopcornTable),
	%% log_set(suturo:'RoboCupDishes', suturo:hasDestinationLocation, PopcornTable),

	%% % mueslibox, crackerbox, milch im regal
	%% forall(is_shelf_layer(ShelfLayer),
	%% 	   (
	%% 		   log_set(suturo:'RoboCupSnacks', suturo:hasOriginLocation, ShelfLayer),
	%% 		   log_set(suturo:'RoboCupSnacks', suturo:hasDestinationLocation, ShelfLayer)
	%% 	   )),

	%% % banane, apfel, früchte auf couch tisch predefined_origin_location
	%% has_urdf_name(CouchTable, 'couch_table:couch_table:table_center'),
	%% log_set(suturo:'RoboCupFruits', suturo:hasOriginLocation, CouchTable),
	%% log_set(suturo:'RoboCupFruits', suturo:hasDestinationLocation, CouchTable),

	ros_info('Finished initializing origin and destination locations for gpsr 2024'),
    !.
init_gpsr_2024 :-
    ros_error('something in init_gpsr_2024 failed, please check what').


log_set_both(Class, Furniture) :-
    log_set(Class, suturo:hasOriginLocation, Furniture),
    log_set(Class, suturo:hasDestinationLocation, Furniture).
