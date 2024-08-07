%% This module loads and creates objects from the semantic map in the database.
:- module(furniture_creation,
	  [
	      load_urdf_from_param(+),
	      init_furnitures/0
	  ]).

:- use_module(library('util/util'),
	      [
            last_element/2,
            ros_error/2,
            ros_warn/2,
            ros_info/2
	      ]).

:- use_module(library('model/object/types'),
	      [
		    is_type/2
	      ]).

:- use_module(library('model/object/object_creation'),
	      [
		    create_object/4
	      ]).

:- use_module(library('semweb/rdf_prefixes'),
	      [
		    rdf_global_id/2
	      ]).

%% load_urdf_from_param(+Param) is semidet.
%
% Is called as inital_goal in the launch file
% Loads the urdf xml data stored in the ros parameter named Param and creates a URDF instance in the database
%
% @param Param Name of the ros parameter containing the urdf xml data
%
load_urdf_from_param(Param):-
    (ros_param_get_string(Param, S) -> % S is the urdf file (a xml file) as a string
	 get_urdf_id(URDF),
	 urdf_load_xml(URDF, S)
    ; ros_error("Error getting ros Parameter for environment urdf")).

%% is_semantic_map_object(?Link) is nondet
%
% True if Link should be loaded for the knowledge semantic map
%
% @param Link Name of a URDF Link as String
%
is_semantic_map_object(Link) :-
    (
    sub_string(Link,_,_,_,"table_center");
    %sub_string(Link,_,_,_,"d_table_origin");
    sub_string(Link,_,_,_,"trashcan_center");
    sub_string(Link,_,_,_,"coathanger_center");
    sub_string(Link,_,_,_,"place_one");
    sub_string(Link,_,_,_,"lamp_center");
    sub_string(Link,_,_,_,"shelf_base_center");
    sub_string(Link,_,_,_,"drawer_front_top");
    sub_string(Link,_,_,_,"drawer_bottom");
    sub_string(Link,_,_,_,"door_center");
    sub_string(Link,_,_,_,"shelf_floor_");
    sub_string(Link,_,_,_,"shelf_layer_");
    sub_string(Link,_,_,_,"shelf_door_");
    sub_string(Link,_,_,_,"bucket_surface_center");
    sub_string(Link,_,_,_,"dishwasher:dishwasher_tray_bottom");
    sub_string(Link,_,_,_,"dishwasher:dishwasher_tray_2_bottom");
    sub_string(Link,_,_,_,"handle"), \+ sub_string(Link,_,_,_,"dishwasher")
    ),
    !.

%% init_furnitures is det.
%
% Reads all relevant objects from the URDF and creates them as their respective instances
%
init_furnitures :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    forall((member(UrdfLink, Links),
	        is_semantic_map_object(UrdfLink)
	       ),
        ignore((
            init_furniture(UrdfLink)
            -> true
            ;  ros_warn("UrdfLink can not be loaded! ~w", [UrdfLink])
            ))),
    ros_info('Semantic map furniture initialized').

%% init_furnitures(?UrdfLink) is semidet.
%
% Creates an object instance of the respective class for the given URDF link and assigns surfaces
%
% @param UrdfLink Urdf link
%
init_furniture(UrdfLink) :-
    ros_info("der UrdfLink: ~w", [UrdfLink]),
    urdf_link_class(UrdfLink, ClassTerm, RobocupName),
    ros_info("der ClassTerm: ~w", [ClassTerm]),
    rdf_global_id(ClassTerm, Class),
    ros_info("de Class: ~w", [Class]),
    furniture_pose(UrdfLink, Pose),
    ros_info("de Pose: ~w", [Pose]),
    furniture_shape(UrdfLink, ShapeTerm),
    ros_info("de shape: ~w", [ShapeTerm]),
    create_object(Furniture, Class, Pose, [shape(ShapeTerm), data_source(semantic_map)]),
    ros_info("Created semantic map object for ~w", [UrdfLink]),
    kb_project((has_urdf_name(Furniture, UrdfLink),
                has_robocup_name(Furniture, RobocupName))),
	% backwards compatibility with table_front_edge_center for planning
	(  atom_concat(Prefix, 'table_center', UrdfLink)
	-> (atom_concat(Prefix, 'table_front_edge_center', ExtraLink),
		kb_project(has_urdf_name(Furniture, ExtraLink)))
	;  true).

furniture_pose(UrdfLink, [ObjectFrame, [0,0,0], [0,0,0,1]]) :-
    atom_concat('iai_kitchen/', UrdfLink, ObjectFrame).

furniture_shape(UrdfLink, ShapeTerm) :-
    get_urdf_id(URDF),
    collision_link(UrdfLink, CollisionLink),
    urdf_link_collision_shape(URDF, CollisionLink, ShapeTerm, _).

%% collision_link(+UrdfLink, -CollisionLink) is semidet.
%
% Helper predicate to define the collision link for a urdf link
%
% @param UrdfLink Name of the objects urdf link as String
% @param CollisionLink Name of the collision link as String
%
collision_link(CollisionLink, CollisionLink) :-
    atom_concat(_, 'table_center', CollisionLink);
    atom_concat(_, 'coathanger_center', CollisionLink);
    atom_concat(_, 'lamp_center', CollisionLink);
    atom_concat(_, 'place_one', CollisionLink);
    atom_concat(_, 'trashcan_center', CollisionLink);
    atom_concat(_, 'door_center', CollisionLink);
    atom_concat(_, 'dishwasher_tray_bottom', CollisionLink);
    atom_concat(_, 'dishwasher_tray_2_bottom', CollisionLink);
    atom_concat(_, 'drawer_bottom', CollisionLink);
    sub_string(CollisionLink,_,_,_,"handle");
    sub_string(CollisionLink,_,_,_,"shelf_layer_");
    sub_string(CollisionLink,_,_,_,"shelf_floor_");
    sub_string(CollisionLink,_,_,_,"shelf_door_").
collision_link(UrdfLink, CollisionLink) :-
    atom_concat(Prefix, '_front_top', UrdfLink),
    atom_concat(Prefix, '_center', CollisionLink).
collision_link(UrdfLink, CollisionLink) :-
    atom_concat(Prefix, 'bucket_surface_center', UrdfLink),
    atom_concat(Prefix, 'bucket_center', CollisionLink).
collision_link(UrdfLink, CollisionLink) :-
    atom_concat(Prefix, 'shelf_base_center', UrdfLink),
    atom_concat(Prefix, 'shelf_back', CollisionLink).

%% urdf_link_class(+UrdfLink, -Class) is semidet.
%
% Returns the owl class of a urdf link based on the name used in the semantic map files.
%
% @param UrdfLink Name of the urdf link as String
% @param Class Class of the urdf link as owl term
%
urdf_link_class(UrdfLink, Class, KnowledgeRole) :-
    atomic_list_concat([_,KnowledgeRole,_], ':', UrdfLink),
    link_role_class(KnowledgeRole, Class),
    !.
urdf_link_class(UrdfLink, Class, KnowledgeRole) :-
    % ignore for handles and so on
    (  atomic_list_concat([_,KnowledgeRole,_], ':', UrdfLink)
    -> true
    ;  KnowledgeRole = undefined),
    split_string(UrdfLink, ":", "", List),
    last_element(List, LinkName),
    link_name_class(LinkName, Class).

:- rdf_meta(link_role_class(+,r)).

%% link_role_class(+KnowledgeRole, -Class) is semidet.
link_role_class(kitchen_table,suturo:'KitchenTable') :- !.
link_role_class(hallway_cabinet,suturo:'HallwayCabinet') :- !.
link_role_class(dining_table,suturo:'DiningTable') :- !.
link_role_class(dinner_table,suturo:'DinnerTable') :- !.
link_role_class(desk,suturo:'Desk') :- !.
link_role_class(coffee_table,suturo:'CoffeeTable') :- !.
link_role_class(kitchen_counter,suturo:'KitchenCounter') :- !.
link_role_class(tv_table,suturo:'TvTable') :- !.
link_role_class(lounge_chair,suturo:'LoungeChair') :- !.

link_role_class(dishwasher_robocup,suturo:'Dishwasher') :- !.


%% link_name_class(+LinkName, -Class) is semidet.
%
% Helper predicate to get the owl class of the last part of the urdf link name used in the semantic map files.
%
% @param LinkName Last part of of the urdf link as String
% @param Class Class of the urdf link type as owl term
%
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"container"),
    Class = soma:'DesignedContainer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"table_center"),
    Class = soma:'Dishwasher',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"door"),
    \+sub_string(LinkName,_,_,_,"handle"),
    Class = soma:'Door',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"drawer"),
    Class = soma:'Drawer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"coathanger_center"),
    Class = suturo:'CoatHanger',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"shelf_layer"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'ShelfLayer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"shelf_floor"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'ShelfLayer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"shelf_base"), % TODO: Fix this inconsistency in the urdf
    Class = soma:'Cupboard',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"table"),
    Class = soma:'Table',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"place_one"),
    Class = suturo:'Couch',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"bucket"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'TrashBin',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"trashcan_center"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'TrashCan',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"dishwasher_tray"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'DishwasherTray',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"handle"),
    Class = soma:'DesignedHandle',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"couch"),
    Class = suturo:'Couch',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"lamp_center"),
    Class = suturo:'Lamp',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"chair_center"),
    Class = suturo:'Chair',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"dishwasher_main"),
    Class = soma:'Dishwasher',
    !.
link_name_class(LinkName, Class) :-
    ros_warn("Unknown link name type: ~w! Using default class soma:DesignedFurniture", [LinkName]),
    Class = soma:'DesignedFurniture',
    !.

