%% The math module contains predicates that provide basic mathematical utils
:- module(math,
	  [
        deg_to_rad/2
	  ]).


%% deg_to_rad(+Degrees, -Radians) is det
%
% Converts Degrees to Radians
%
% @param Degrees The number of degrees to convert
% @param Radians The number of radians
%
deg_to_rad(Degrees, Radians) :-
    % Define the conversion factor from degrees to radians (pi/180)
    DegreesToRadiansConversionFactor = 0.0174533,
    Radians is Degrees * DegreesToRadiansConversionFactor.