%% robot module
%  This module contains anything related to the robot.
:- module(robot,
    [
      robot_location(-)
    ]).

%% robot_location(-Location) is det.
%
% Returns the current location [X, Y, Z] of the robot (base_footprint), relative to the map origin.
%
% @param Location The current location of the robot.
%
robot_location(Location) :-
    get_urdf_origin(Origin),
    kb_call(is_at(base_footprint, [Origin, Location, _])).