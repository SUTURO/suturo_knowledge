%% size reasoner
% This module contains predicates for size reasoning. 
% Anything related to the size, proportions or dimensions of objects should be in this module.
:- module(size,
    [
      is_tiny(r)
    ]).

%% is_tiny(+Object) is semidet.
%
% True if the object is tiny according to the RoboCup rulebook.
% The RoboCup rulebook states that objects with any side smaller than 5cm are considered tiny.
%
% @param Object The object to check.
%
is_tiny(Object) :-
	object_shape_workaround(Object,_,ShapeTerm,_,_),
    ( ShapeTerm = box(X,Y,Z),
        (
            X =< 0.05 ;
            Y =< 0.05 ;
            Z =< 0.05
        )
    ; ShapeTerm = cylinder(Radius,Length),
        (
            Radius =< 0.025 ;
            Length =< 0.05
        )
    ; ShapeTerm = sphere(Radius),
        (
            Radius =< 0.025
        )
    ; 	ShapeTerm = mesh(File, Scale),
        ( 
            false
        )
    ).