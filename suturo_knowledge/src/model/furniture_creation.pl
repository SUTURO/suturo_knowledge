%% This module loads and creates furniture in the database.
%
% Additionally it currently has predicates used in load_urdf_from_param/1 and init_furnitures/0 that should be moved somewhere else as soon as we have time.
:- module(furniture_creation,
	  [
	      create_table/2,
	      load_urdf_from_param/1,
	      init_furnitures/0,
	      has_urdf_name/2,
	      has_tf_name/2
	  ]).

:- use_module(furniture_types,
	      [
		  is_type/2,
		  is_table/1,
		  is_drawer/1
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
%    sub_string(Link,_,_,_,"shelf_base_center");
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
    split_string(FurnitureLink, ":", "", [_, Type, Shape]),
    create_furniture(Type, Furniture),
    % Workaround: furniture that doesn't have a type and couldn't get created shouldn't have a urdf name.
    (var(Furniture) -> true;
     kb_project(has_urdf_name(Furniture, FurnitureLink)),
     % TODO: don't hardcode the shape
     [Depth, Width, Height] = [1,1,1],
     kb_project(is_shape(Shape)),
     kb_project(is_boxShape(ShapeRegion)),
     kb_project(holds(Furniture, soma:hasShape, Shape)),
     kb_project(holds(Shape, dul:hasRegion, ShapeRegion)),
     % Doesn't use object_dimensions/4 because it throws an exception
     kb_project(holds(ShapeRegion, soma:hasDepth, Depth)),
     kb_project(holds(ShapeRegion, soma:hasWidth, Width)),
     kb_project(holds(ShapeRegion, soma:hasHeight, Height))
    ).
    % TODO not implemented yet
    %assign_furniture_location(Furniture),
    %assign_surfaces(Furniture, FurnitureLink, Shape).

%% has_urdf_name(?Object, ?URDFName) is nondet.
%
% Looks up which object has which urdfname
% works in both directions
%
% part of the planning interface.
% the api should stay stable.
has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).

%% has_tf_name(Object, TFName)
%
% gets the tf name of an object.
% for objects that have a urdf name, the TFName is based on the urdf name.
% for other objects, it is the part after the #
%
% part of the planning interface.
% the api should stay stable.
has_tf_name(Object, TFName) :-
    % anything with a # is an object and not a urdf name
    sub_string(Object, _, _, After, "#"),
    (
	has_urdf_name(Object, URDFName) ->
	has_tf_name(URDFName, TFName);
	% for stuff that doesn't have a urdf name, use the last part of the iri
	sub_atom(Object, _, After, 0, TFName)
    ),
    !.

% has_tf_name for urdf names
has_tf_name(URDFName, TFName) :-
    % using a not here so the cut 3 lines above is a green cut.
    not(sub_string(URDFName, _, _, _, "#")),
    % TODO don't hardcode iai_kitchen
    atom_concat('iai_kitchen/', URDFName, TFName).

%% create_furniture(+FurnitureType, -Furniture)
%
% create a furniture iri and project the type based on the link name
create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    kb_project(is_table(Furniture)),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"drawer"),
    kb_project(is_drawer(Furniture)),
    !.

create_furniture(_, _) :-
    % Ignore unknown furniture for now
    true.

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
