% This folder contains anything relevant to modelling the world around the robot.
% This means objects and information that were gathered during runtime and information that are semi-hardcoded in the semantic map.

:- ensure_loaded('urdf').
:- ensure_loaded('robot').
:- use_directory('object').
:- use_directory('semantic_map').