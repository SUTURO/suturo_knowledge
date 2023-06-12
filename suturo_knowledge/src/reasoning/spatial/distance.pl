%% distance reasoner
% This module contains predicates for calculating the distance between objects or points.
:- module(distance,
	  [
        euclidean_distance(+, +, -)
	  ]).

%% euclidean_distance(+Point1, +Point2, -Distance) is det.
%
% Calculates the euclidean distance between two points.
%
% @param Point1 The first point [X,Y,Z].
% @param Point2 The second point [X,Y,Z].
% @param Distance The distance between the two points.
%
euclidean_distance([X1, Y1, Z1], [X2, Y2, Z2], Distance) :-
	XDiff is X1 - X2, YDiff is Y1 - Y2, ZDiff is Z1 - Z2,
	XSquare is XDiff * XDiff,
	YSquare is YDiff * YDiff,
	ZSquare is ZDiff * ZDiff,
	Sum is XSquare + YSquare + ZSquare,
	Distance is sqrt(Sum).