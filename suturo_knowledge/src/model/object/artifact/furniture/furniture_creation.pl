%% This module loads and creates designed furniture in the database.
:- module(furniture_creation,
	  [
	      load_urdf_from_param(+),
	      init_furnitures/0
	  ]).

:- use_module(library('util/util'),
	      [
		  has_urdf_name/2,
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

%% load_urdf_from_param(+Param) is det.
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
    ; ros_warn("Error getting ros Parameter for environment urdf")).

%% is_furniture_link(?Link) is nondet
%
% True if Link is a link of a furniture in the URDF
%
% @param Link Name of a URDF Link as String
%
is_furniture_link(Link) :-
    ros_info("~w", [Link]),
    sub_string(Link,_,_,_,"table_front_edge_center");
    sub_string(Link,_,_,_,"shelf_base_center");
    sub_string(Link,_,_,_,"drawer_front_top").

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
    split_string(FurnitureLink, ":", "", [_, Type, _]),
    furniture_class(Type, ClassTerm),
    % The rdf meta only does compile-time expansion of soma:'Table' and friends.
    % Because of that, the expansion is done explicitly.
    % TODO check if there is a better method for this.
    rdf_global_id(ClassTerm, Class),
    furniture_pose(FurnitureLink, Pose),
    furniture_shape(FurnitureLink, ShapeTerm),
    create_object(Furniture, Class, Pose, [shape(ShapeTerm), data_source(semantic_map)]),
    kb_project(has_urdf_name(Furniture, FurnitureLink)).

furniture_pose(FurnitureLink, [FurnitureFrame, [0,0,0], [0,0,0,1]]) :-
    atom_concat('iai_kitchen/', FurnitureLink, FurnitureFrame).

furniture_shape(FurnitureLink, ShapeTerm) :-
    get_urdf_id(URDF),
    collision_link(FurnitureLink, CollisionLink),
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

%% furniture_class(+FurnitureType, -FurnitureClass) is det.
%
% get the owl class of a furniture type based on the name used in the semantic map files.
% TODO: bucket, container, tray
furniture_class(FurnitureType, FurnitureClass) :-
    sub_string(FurnitureType,_,_,_,"container"),
    FurnitureClass = soma:'DesignedContainer',
    !.

furniture_class(FurnitureType, FurnitureClass) :-
    sub_string(FurnitureType,_,_,_,"drawer"),
    FurnitureClass = soma:'Drawer',
    !.

furniture_class(FurnitureType, FurnitureClass) :-
    sub_string(FurnitureType,_,_,_,"shelf"),
    FurnitureClass = soma:'Shelf',
    !.

furniture_class(FurnitureType, FurnitureClass) :-
    sub_string(FurnitureType,_,_,_,"table"),
    FurnitureClass = soma:'Table',
    !.

furniture_class(FurnitureType, FurnitureClass) :-
    ros_warn("Unknown furniture type ~w", [FurnitureType]),
    FurnitureClass = soma:'DesignedFurniture',
    !.
