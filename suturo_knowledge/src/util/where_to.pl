%% The whereTo module reads the String of a location and gives back the pose.

%:- use_module(library('rosprolog')).
:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).

:- module(where_to,
	  [
        where_to(+, -)
	  ]).


% callback function for subscriber 
%% whereTo(+String, -PoseStamped)
whereTo(String, PoseStamped) :-
	PoseStamped = ['map', [2.94,3.80,0], [0.0,0.0,0.0,1.0]].

    %atom_concat('soma:', String, Type),
	%has_type(Object, Type),
	%object_pose(Object, PoseStamped).


	