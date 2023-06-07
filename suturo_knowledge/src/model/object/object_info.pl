%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
      	object_pose(r,-),
		tiny_object(r),
		is_suturo_object(r),
		set_object_handled(r),
		set_object_not_handled(r),
		update_handle_state(r,+),
		handled(r),
		not_handled(r),
		objects_not_handled(-)
	  ]).

:- use_module(library('ros/tf/tf'),
	      % actually uses tf:tf_get_pose, but that is not exported by tf
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
%
% Get the pose of an object.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).

%% tiny_object(+Object) is semidet.
%
% True if the object is tiny according to the RoboCup rulebook.
% The RoboCup rulebook states that objects with any side smaller than 5cm are considered tiny.
%
% @param Object The object to check.
%
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

%% is_suturo_object(+Object) is semidet.
%
% True if the object is a Suturo object.
% An object is a Suturo object if its a physical object and has a data source. 
%
% @param Object The object to check.
%
is_suturo_object(Object):-
	is_physical_object(Object),
	triple(Object, suturo:'hasDataSource', DataSource).

%% is_physical_object(+Object) is det.
%
% Sets the state of the object to handled.
%
% @param Object The object to set the state of.
%
set_object_handled(Object) :-
	update_handle_state(Object, true).
	
%% is_physical_object(+Object) is det.
%
% Sets the state of the object to not handled.
%
% @param Object The object to set the state of.
%
set_object_not_handled(Object) :-
	update_handle_state(Object, false).

%% update_handle_state(+Object, +State) is det.
%
% Updates the handle state of an object.
%
% @param Object The object to update the handle state of.
% @param State The new state of the object. (true or false)
%
update_handle_state(Object, State) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	forall(triple(HandleState, suturo:'handled', OldValue),
		kb_unproject(triple(HandleState, suturo:'handled', OldValue))),
    kb_project(triple(HandleState, suturo:'handled', State)).

%% handled(+Object) is semidet.
%
% True if the object is handled.
%
% @param Object The object to check.
%
handled(Object) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', true).

%% not_handled(+Object) is semidet.
%
% True if the object is not handled.
%
% @param Object The object to check.
%
not_handled(Object) :-
	is_suturo_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', false).

%% objects_not_handled(-Objects) is det.
%
% Returns a list of all objects that are not handled.
%
% @param Objects The list of objects that are not handled.
%
objects_not_handled(Objects):-
    findall(Object,
    (
        is_suturo_object(Object),
        not_handled(Object)
    ),
    Objects).