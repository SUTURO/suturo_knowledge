%% This module loads and creates designed furniture in the database.
%
% Additionally it currently has predicates used in load_urdf_from_param/1 and init_furnitures/0 that should be moved somewhere else as soon as we have time.
:- module(furniture_creation,
	  [
	      create_table(-,+),
	      load_urdf_from_param(+),
	      init_furnitures/0
	  ]).

:- use_module(library('util/util'),
	      [
		  has_urdf_name/2,
		  has_tf_name/2,
		  ros_warn/2
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

%% is_furniture_link(?Link) is nondet
%
% True if Link is a link of a furniture in the URDF
%
% @param Link Name of a URDF Link as String
%
is_furniture_link(Link) :-
    sub_string(Link,_,_,_,"table_front_edge_center");
    sub_string(Link,_,_,_,"shelf_base_center");
%    sub_string(Link,_,_,_,"bucket_front_edge_center");
    sub_string(Link,_,_,_,"drawer_front_top").



%%
% is called as inital_goal in the launch file
% loads the urdf xml data stored in the ros parameter named Param
load_urdf_from_param(Param):-
    (ros_param_get_string(Param,S) -> % S is the urdf file (a xml file) as a string
	 get_urdf_id(URDF),
	 urdf_load_xml(URDF,S)
    ; ros_warn("Error getting ros Parameter for environment urdf")).
    % set_pose_origin hangs because i can't supply the load_rdf option to urdf_load_xml/2
    % get_urdf_origin(Origin),
    % urdf_set_pose_to_origin(URDF,Origin).
    % was in the old suturo code, looks like it doesn't do anything useful
    %urdf_link_names(URDF,Links).

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
    %% create_furniture(Type, Furniture),
    %% % Workaround: furniture that doesn't have a type and couldn't get created shouldn't have a urdf name.
    %% atom(Furniture),
    %% kb_project(has_urdf_name(Furniture, FurnitureLink)),
    %% assign_furniture_location(Furniture, FurnitureLink),
    %% assign_furniture_shape(Furniture, FurnitureLink).

furniture_pose(FurnitureLink, [FurnitureFrame, [0,0,0], [0,0,0,1]]) :-
    atom_concat('iai_kitchen/', FurnitureLink, FurnitureFrame).

assign_furniture_location(Furniture, FurnitureLink) :-
    universal_scope(Scope),
    atom_concat('iai_kitchen/', FurnitureLink, FurnitureFrame),
    tf_set_pose(Furniture, [FurnitureFrame, [0,0,0], [0,0,0,1]], Scope).

furniture_shape(FurnitureLink, ShapeTerm) :-
    get_urdf_id(URDF),
    collision_link(FurnitureLink, CollisionLink),
    urdf_link_collision_shape(URDF, CollisionLink, ShapeTerm, _).

assign_furniture_shape(Furniture, FurnitureLink) :-
    get_urdf_id(URDF),
    collision_link(FurnitureLink, CollisionLink),
    urdf_link_collision_shape(URDF, CollisionLink, ShapeTerm, _),
    (box(Depth, Width, Height) = ShapeTerm ->
	 true;
     ros_warn('Shape is not a box: ~w~n From link ~w', [ShapeTerm, CollisionLink]),
     [Depth, Width, Height] = [1,1,1]),
    kb_project(is_shape(Shape)),
    kb_project(is_boxShape(ShapeRegion)),
    kb_project(holds(Furniture, soma:hasShape, Shape)),
    kb_project(holds(Shape, dul:hasRegion, ShapeRegion)),
    % Doesn't use object_dimensions/4 because it throws an exception
    kb_project(holds(ShapeRegion, soma:hasDepth, Depth)),
    kb_project(holds(ShapeRegion, soma:hasWidth, Width)),
    kb_project(holds(ShapeRegion, soma:hasHeight, Height)).

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

%% create_furniture(+FurnitureType, -Furniture)
%
% create a furniture iri and project the type based on the link name
% TODO: bucket, container, tray

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"container"),
    kb_project(is_type(Furniture, soma:'DesignedContainer')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"drawer"),
    kb_project(is_drawer(Furniture)),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"shelf"),
    kb_project(is_shelf(Furniture)),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    kb_project(is_table(Furniture)),
    !.

create_furniture(FurnitureType, Furniture) :-
    ros_warn("Unknown furniture type ~w", [FurnitureType]),
    kb_project(new_iri(Furniture, soma:'DesignedFurniture')),
    !.

%% create_table(-Table, +Dimensions)
%
% directly create a table with the specified dimensions.
%
% @param Dimensions is a list of Depth, Width, and Height
create_table(Table, [Depth, Width, Height]) :-
    kb_project(is_table(Table)),
    kb_project(is_shape(Shape)),
    kb_project(is_boxShape(ShapeRegion)),
    kb_project(holds(Table, soma:hasShape, Shape)),
    kb_project(holds(Shape, dul:hasRegion, ShapeRegion)),
    % Doesn't use object_dimensions/4 because it throws an exception
    kb_project(holds(ShapeRegion, soma:hasDepth, Depth)),
    kb_project(holds(ShapeRegion, soma:hasWidth, Width)),
    kb_project(holds(ShapeRegion, soma:hasHeight, Height)).

is_shape(Shape) ?+>
    is_type(Shape, soma:'Shape').

is_boxShape(ShapeRegion) ?+>
    is_type(ShapeRegion, soma:'BoxShape').
