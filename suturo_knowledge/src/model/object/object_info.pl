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


tiny_object(Object) :-
	object_shape_workaround(Object,_,ShapeTerm,_,_),
(
	ShapeTerm = box(X,Y,Z),
	(
		X =< 0.05;
		Y =< 0.05;
		Z =< 0.05
	);
	ShapeTerm = cylinder(Radius,Length),
	(
		Radius =< 0.025;
		Length =< 0.05
	);
	ShapeTerm = sphere(Radius),
	(
		Radius =< 0.025
	);
	ShapeTerm = mesh(File, Scale),
	(
		false
	)
).


is_suturo_object(Object):-
	is_physical_object(Object),
	triple(Object, suturo:'hasDataSource', DataSource).

set_object_handled(Object) :-
	update_handle_state(Object, true).
	
set_object_not_handled(Object) :-
	update_handle_state(Object, false).

update_handle_state(Object, State) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	forall(triple(HandleState, suturo:'handled', OldValue),
		kb_unproject(triple(HandleState, suturo:'handled', OldValue))),
    kb_project(triple(HandleState, suturo:'handled', State)).

handled(Object) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', true).

not_handled(Object) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', false).

objects_not_handled(Objects):-
    findall(Object,
    (
        is_suturo_object(Object),
        not_handled(Object)
    ),
    Objects).