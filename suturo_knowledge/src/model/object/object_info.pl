%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
      	object_pose(r,-),
		tiny_object(r),
		is_perceived_object(r),
		set_object_handled(r),
		set_object_not_handled(r),
		update_handle_state(r,+),
		handled(r),
		not_handled(r),
		objects_not_handled(-),
		predefined_origin_location(r,-),
	  	predefined_destination_location(r,-)
	  ]).

:- use_module(library('ros/tf/tf')).

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

%% is_perceived_object(+Object) is semidet.
%
% True if the object is a physical object and has the a data source 'perception'. 
%
% @param Object The object to check.
%
is_perceived_object(Object):-
	is_physical_object(Object),
	holds(Object, suturo:'hasDataSource', perception).

%% set_object_handled(+Object) is det.
%
% Sets the state of the object to handled.
%
% @param Object The object to set the state of.
%
set_object_handled(Object) :-
	update_handle_state(Object, true).
	
%% set_object_not_handled(+Object) is det.
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
	is_perceived_object(Object),
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
	is_perceived_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', true).

%% not_handled(+Object) is semidet.
%
% True if the object is not handled.
%
% @param Object The object to check.
%
not_handled(Object) :-
	is_perceived_object(Object),
	triple(Object, suturo:'hasHandleState', HandleState),
	triple(HandleState, suturo:'handled', false).

%% objects_not_handled(-Objects) is det.
%
% Returns a list of all perceived objects that are not handled.
%
% @param Objects The list of objects that are not handled.
%
objects_not_handled(Objects):-
    findall(Object,
    (
        is_perceived_object(Object),
        not_handled(Object)
    ),
    Objects).

%% predefined_origin_location(+Class, -OriginLocation) is semidet.
%
% Get the predefined origin location of an object class.
% The OriginLocation is the location (reference object) where the object is placed at the beginning of the task.
% 
% @param Class The IRI or abbreviated name of the class. 
% @param OriginLocation The predefined origin location.
%
predefined_origin_location(Class, OriginLocation) :-
    holds(Class, suturo:hasOriginLocation, OriginLocation).
predefined_origin_location(Class, OriginLocation) :-
	subclass_of(Class, dul:'PhysicalObject'),
    holds(dul:'PhysicalObject', suturo:hasOriginLocation, OriginLocation).

%% predefined_destination_location(+Class, -DestinationLocation) is semidet.
%
% Get the predefined destination location of an object class.
% The DestinationLocation is the location (reference object) where the object should placed at the end of the task.
% 
% @param Class The IRI or abbreviated name of the class. 
% @param DestinationLocation The predefined destination location.
%
predefined_destination_location(Class, DestinationLocation) :-
    holds(Class, suturo:hasDestinationLocation, DestinationLocation).
predefined_destination_location(Class, DestinationLocation) :-
	subclass_of(Class, dul:'PhysicalObject'),
    holds(dul:'PhysicalObject', suturo:hasDestinationLocation, DestinationLocation).