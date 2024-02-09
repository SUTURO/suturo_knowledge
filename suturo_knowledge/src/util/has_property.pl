%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
        is_fragile(r),
		what_object(+,r),
		fragility_new(+)
	  ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% go up all superclasses of an object till you find a superclass with property 'Fragility' 
%% is_fragile(r Object)
is_fragile(Object) :-
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, B,suturo:'Fragility').

%% what_object(+ ObjName, r Object)
what_object(ObjName, Object) :-
	triple(O,_, suturo:hasPredefinedName), triple(O, owl:hasValue, ObjName), triple(Object,_,O), 
	% if one exists, use only that
	!.


% objName = 'metal bowl'
%% fragility_new(r ObjName)
fragility_new(ObjName) :-
	triple(O,_, suturo:hasPredefinedName), triple(O, owl:hasValue, ObjName), triple(Object,_,O),  
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Asking for the Positions of furniture