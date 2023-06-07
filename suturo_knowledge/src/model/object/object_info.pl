%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
      	object_pose(r,-),
		tiny_object/1,
		is_suturo_object/1,
		set_object_handled/1,
		set_object_not_handled/1,
		update_handle_state/2,
		handled/1,
		not_handled/1,
		objects_not_handled/1
	  ]).

:- use_module(library('ros/tf/tf'),
	      % actually uses tf:tf_get_pose, but that is not exported by tf
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
%
% Get the pose of an object.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).
