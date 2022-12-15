module(pose,
       [
	   object_pose/2
       ]).

:- use_module(library('ros/tf/tf'),
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).
