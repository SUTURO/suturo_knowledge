%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
        %is_fragile(r),
		what_object(+,r),
		fragility_new(+),
		is_perishable(+),
		have_same_class(+,+),
		preorlo_check(r, -),
		grasp_pose(+,-),
		has_position(+,-),
		has_value(+,r,-),
		middle(+,-),
		exit_pose(+,-),
		entry_pose(+,-),
		path(?,?)
	  ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% go up all superclasses of an object till you find a superclass with property 'Fragility' 
%% is_fragile(r Object)
%is_fragile(Object) :-
%	triple(Object, transitive(rdfs:'subClassOf'), X),
%	triple(X, B,suturo:'Fragility').

%% fragility_new(r ObjName)
fragility_new(ObjName) :-
	triple(O,_, suturo:hasPredefinedName), 
	triple(O, owl:hasValue, ObjName), 
	triple(Object,_,O),  
	%triple(Object, transitive(rdfs:'subClassOf'), X),
	%triple(X, B, suturo:'Fragility').

	transitivee(Object).

%transitivee(r Object)
transitivee(Object) :- 
	triple(Object, B, suturo:'Fragility').

%transitivee(r Object)
transitivee(Object) :-
	subclass_of(Object, X),
	transitivee(X).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
%% is_perishable(+ObjName)
% 
% ask if object is perishable
is_perishable(ObjName):-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, B, suturo:'Perishable').


%% preorlo_check(r, -)
preorlo_check(ObjName, Objectt):-
	what_object(ObjName, Object),
	triple(O,_, suturo:hasOriginLocation),
	triple(Object, owl:onProperty, X), 
	triple(Object,_,O),
	!.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% what_object(+ ObjName, r Object)
%
% get the Object that has the predefined name "ObjName"
what_object(ObjName, Object) :-
	triple(O,_, suturo:hasPredefinedName),
	triple(O, owl:hasValue, ObjName), 
	triple(Object,_,O), 
	% if one exists, use only that
	!.

%% have_same_class(+, +)
%
% check, if two objects belong to the same class
have_same_class(ObjName1, ObjName2) :-
	what_object(ObjName1, X),
	what_object(ObjName2, Y),
	subclass_of(X, Z),
	subclass_of(Y, Z),
	!.

grasp_pose(ObjName , Pose) :-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, suturo:hasGraspPose),
	triple(X, owl:hasValue, Pose), 
	% if one exists, use only that
	!.

has_position(ObjName, PoseStamped):-
	what_object(ObjName, Object), 
	triple(Object, transitive(rdfs:'subClassOf'), Q),
	triple(Q, _, suturo:hasPosition),
	triple(Q, owl:hasValue, Pose), 
	has_type(Plate, soma:'Plate'),
	object_pose(Plate, [Frame, [X,Y,Z] , Rotation]),
	( Pose == 'right'
	-> NewY is Y - 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'left'
	-> NewY is Y + 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'top_right'
	-> NewX is X - 0.2,  NewY is Y + 0.2,
		PoseStamped = [Frame, [NewX,NewY,Z] , Rotation]
	).
	
has_value(ObjName, Property, Value) :-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, Property),
	triple(X, owl:hasValue, Value).


connected_rooms(kitchen,living_room).
connected_rooms(living_room, dining_room).

path(X,Y):- connected_rooms(X,Y).
path(X,Y):- connected_rooms(Y,X).
path(X,Y):- connected_rooms(X,Z), path(Z,Y).
path(X,Y):- connected_rooms(Z,X), path(Z,Y).


middle(kitchen, [map, [2.87, -1.11, 0] , [0, 0, 0, 1.0]]).
middle(living_room, [map, [2.94, 2.62, 0] , [0, 0, 0, 1.0]]).
middle(dining_room, [map, [2.88, 4.9, 0] , [0, 0, 0, 1.0]]).

entry_pose(kitchen, [map, [0.37, 0.01, 0] , [0, 0, 0, 1.0]]).
entry_pose(living_room, [map, [2.34, 2.71, 0] , [0, 0, 0, 1.0]]).
entry_pose(dining_room, [map, [0, 3.74, 0] , [0, 0, 0, 1.0]]).

exit_pose(kitchen, [map, [1.2, -0.218, 0] , [0, 0, 0, 1.0]]).
exit_pose(living_room, [map, [3.02, 2.53, 0] , [0, 0, 0, 1.0]]).
exit_pose(dining_room, [map, [0.73, 3.77, 0] , [0, 0, 0, 1.0]]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% get_pose(+ ObjName, r Object)
%