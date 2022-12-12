% This folder contains most stuff relevant to modelling the world around the robot.
% This means objects that were percieved during runtime and furniture that is semi-hardcoded in the semantic map.

% make sure types and util is loaded before furniture and object since they use is_type from here.
:- ensure_loaded('types').

:- use_directory('furniture').
:- use_directory('object').
