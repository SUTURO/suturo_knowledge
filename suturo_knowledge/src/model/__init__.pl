% This folder contains most stuff relevant to modelling the world around the robot.
% This means objects that were percieved during runtime and objets that is semi-hardcoded in the semantic map.

% make sure types and util is loaded before artifact and object since they use is_type from here.
:- ensure_loaded('types').

:- ensure_loaded('pose').

:- use_directory('object').
