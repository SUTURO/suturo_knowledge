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
		has_position(+,r,-),
		has_value(+,r,-),
		place_plate(r,-)
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

has_position(ObjName, Table, PoseStamped):-
	what_object(ObjName, Object), 
	triple(Object, transitive(rdfs:'subClassOf'), Q),
	triple(Q, _, suturo:hasPosition),
	triple(Q, owl:hasValue, Pose), 
	place_plate(Table, [Frame, [X,Y,Z] , Rotation]),
	( Pose == 'right'
	-> NewY is Y - 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'left'
	-> NewY is Y + 0.2, 
		PoseStamped = [Frame, [X,NewY,Z] , Rotation]
	; Pose == 'top_right'
	-> NewX is X + 0.2,  NewY is Y - 0.2,
		PoseStamped = [Frame, [NewX,NewY,Z] , Rotation]
	).
	
has_value(ObjName, Property, Value) :-
	what_object(ObjName, Object),
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, _, Property),
	triple(X, owl:hasValue, Value).

% table muss hier als soma:Table_XYZ stehen 


place_plate(Table, TPose):-
	object_pose(Table, [Frame, [X,Y,Z], Rotation]),
    longest_side(Table, YSize),
	writeln(YSize),
	YNew is (Y + (YSize / 2.0)) - 0.3,
	shortest_side(Table, XSize),
	XNew is (X - XSize/ 2.0) + 0.2,
	writeln(YNew),
	TPose = [Frame, [XNew,YNew,Z], [0,0,0,1.0]].
