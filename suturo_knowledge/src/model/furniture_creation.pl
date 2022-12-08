%% This module loads and creates furniture in the database.
:- module(furniture_creation,
	  [
	      create_table/2,
	      load_urdf_from_param/1,
	      init_furnitures/0
	  ]).

:- use_module(furniture_types,
	      [
		  is_type/2,
		  is_table/1
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



/**
* is called as inital_goal in the launch file
*/
load_urdf_from_param(Param):-
    ros_param_get_string(Param,S), % S is the urdf file (a xml file) as a string
    get_urdf_id(URDF),
    urdf_load_xml(URDF,S),
    get_urdf_origin(Origin).
    % set_post_origin hangs
    %urdf_set_pose_to_origin(URDF,Origin).
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
    forall((
		  member(FurnitureLink, Links),
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
    kb_project(has_urdf_name(Furniture, FurnitureLink)).
    %assign_furniture_location(Furniture),
    %assign_surfaces(Furniture, FurnitureLink, Shape).

has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).

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

has_tf_name(URDFName, TFName) :-
    not(sub_string(URDFName, _, _, _, "#")),
    % TODO don't hardcode iai_kitchen
    atom_concat('iai_kitchen/', URDFName, TFName).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    kb_project(is_table(Furniture)),
    !.

create_furniture(_, _) :-
    % Ignore unknown furniture for now
    true.

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
