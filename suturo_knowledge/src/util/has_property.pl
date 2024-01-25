%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
        is_fragile(r),
		gib_object(+,r)
	  ]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% go up all superclasses of an object till you find a superclass with property 'Fragility' 
%is_fragile(r Object)
is_fragile(Object) :-
	triple(Object, transitive(rdfs:'subClassOf'), X),
	triple(X, B,suturo:'Fragility').

what_object(Name, Object) :-
	kb_call(holds(Object, suturo:hasPredifinedName, Name)),
	is_fragile(Name).