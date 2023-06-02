%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
      	object_pose(r,-),
      	object_predefined_destination_location(r,-)
	  ]).

:- use_module(library('ros/tf/tf'),
	      % actually uses tf:tf_get_pose, but that is not exported by tf
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
%
% Get the pose of an object.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).

%% object_predefined_origin_location(+Object, -PoseStamped) is semidet.
%
% Get the predefined origin location of an object.
% 
% @param Object The object.
% @param PoseStamped The pose of the predefined origin location.
%
object_predefined_origin_location(Object, PoseStamped) :-
    false.

%% object_predefined_destination_location(+Object, -PoseStamped) is semidet.
%
% Get the predefined destination location of an object.
% 
% @param Object The object.
% @param PoseStamped The pose of the predefined destination location.
%
object_predefined_destination_location(Object, PoseStamped) :-
    false.
