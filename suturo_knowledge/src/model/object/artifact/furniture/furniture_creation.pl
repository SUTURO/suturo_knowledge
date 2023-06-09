%% This module loads and creates designed furniture in the database.
:- module(furniture_creation,
	  [
	      load_urdf_from_param(+),
	      init_furnitures/0
	  ]).

:- use_module(library('util/util'),
	      [
            has_urdf_name/2,
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

get_urdf_id(URDF) :-
    URDF = arena.

get_urdf_origin(Origin) :-
    Origin = map.

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

%% is_furniture_link(?Link) is nondet
%
% True if Link is a link of a furniture in the URDF
%
% @param Link Name of a URDF Link as String
%
is_furniture_link(Link) :-
    (
        sub_string(Link,_,_,_,"table_front_edge_center");
        sub_string(Link,_,_,_,"shelf_base_center");
        sub_string(Link,_,_,_,"drawer_front_top");
        sub_string(Link,_,_,_,"door_center");
        sub_string(Link,_,_,_,"shelf_floor_");
        sub_string(Link,_,_,_,"shelf_door_");
        sub_string(Link,_,_,_,"bucket_center")
    ), % TODO: We exclude handles for now. They dont have consistent urdf link names
    \+ sub_string(Link,_,_,_,"handle").

%% init_furnitures is nondet
%
% Reads all furnitures from the URDF and creates an
% instance of type soma:'DesignedFurniture' for each
%
init_furnitures :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    forall((member(FurnitureLink, Links),
	    is_furniture_link(FurnitureLink)
	   ),
	   init_furniture(FurnitureLink)).

%% init_furnitures(?FurnitureLink) is nondet
%
% Creates a furniture instance of type soma:'DesignedFurniture'
% and assigns surfaces
%
% @param FurnitureLink String of URDF Link name
%
init_furniture(FurnitureLink) :-
    urdf_link_class(FurnitureLink, ClassTerm),
    rdf_global_id(ClassTerm, Class),
    ros_info('Class ~w', [Class]),
    furniture_pose(FurnitureLink, Pose),
    furniture_shape(FurnitureLink, ShapeTerm),
    create_object(Furniture, Class, Pose, [shape(ShapeTerm), data_source(semantic_map)]),
    kb_project(has_urdf_name(Furniture, FurnitureLink)).

furniture_pose(FurnitureLink, [FurnitureFrame, [0,0,0], [0,0,0,1]]) :-
    atom_concat('iai_kitchen/', FurnitureLink, FurnitureFrame).

furniture_shape(FurnitureLink, ShapeTerm) :-
    get_urdf_id(URDF),
    ros_info('URDF ~w', [URDF]),
    collision_link(FurnitureLink, CollisionLink),
    ros_info('CollisionLink ~w', [CollisionLink]),
    urdf_link_collision_shape(URDF, CollisionLink, ShapeTerm, _).

collision_link(FurnitureLink, CollisionLink) :-
    atom_concat(Prefix, '_front_edge_center', FurnitureLink),
    atom_concat(Prefix, '_center', CollisionLink).

collision_link(FurnitureLink, CollisionLink) :-
    atom_concat(Prefix, '_front_top', FurnitureLink),
    atom_concat(Prefix, '_center', CollisionLink).

collision_link(FurnitureLink, CollisionLink) :-
    atom_concat(Prefix, 'shelf_base_center', FurnitureLink),
    atom_concat(Prefix, 'shelf_back', CollisionLink).

%% urdf_link_class(+UrdfLink, -Class) is semidet.
%
% Returns the owl class of a furniture link based on the name used in the semantic map files.
%
% @param UrdfLink Name of the furniture link as String
% @param Class Class of the furniture link as owl term
%
urdf_link_class(UrdfLink, Class) :-
    split_string(UrdfLink, ":", "", List),
    last_element(List, LinkName),
    link_name_class(LinkName, Class).

%% link_name_class(+LinkName, -Class) is semidet.
%
% Helper predicate to get the owl class of the last part of the urdf link name used in the semantic map files.
% 
% @param LinkName Last part of of the urdf link as String
% @param Class Class of the furniture type as owl term
%
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"container"),
    Class = soma:'DesignedContainer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"door"),
    Class = soma:'Door',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"drawer"),
    Class = soma:'Drawer',
    !.
link_name_class(LinkName, Class) :-
    sub_string(LinkName,_,_,_,"shelf_floor"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'Shelf',
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
    sub_string(LinkName,_,_,_,"bucket"), % TODO: Fix this inconsistency in the urdf
    Class = suturo:'TrashBin',
    !.
link_name_class(LinkName, Class) :-
    ros_warn("Unknown link name type: ~w! Using default class soma:DesignedFurniture", [LinkName]),
    Class = soma:'DesignedFurniture',
    !.