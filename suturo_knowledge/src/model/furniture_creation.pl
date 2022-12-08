%% This module loads and creates furniture in the database.
:- module(furniture_creation,
	  [
	      create_table/2
	  ]).

:- use_module(furniture_types,
	      [
		  is_type/2,
		  is_table/1
	      ]).

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
