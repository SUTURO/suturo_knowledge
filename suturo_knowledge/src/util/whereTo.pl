%% The whereTo module reads the String of a location and gives back the pose.
:- use_module(library('rosprolog')).
%:- initialization(ros_init('nnode', [], [])).

:- module(whereTo,
	  [
        whereTo(+, -)
	  ]).

% callback funct für den subscriber
%% whereTo(+String, -PoseStamped)
whereTo(String, PoseStamped) :-
	PoseStamped = ['map', [2.94,3.80,0], [0.0,0.0,0.0,1.0]].
    %atom_concat('http://www.ease-crc.org/ont/SOMA.owl#', String, Type),
	%has_type(Object, Type),
	%object_pose(Object, PoseStamped).


	