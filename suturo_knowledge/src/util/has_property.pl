%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
		create_bowl(+),
        has_property(+),
		has_propertyCerealBowl(+),
		test(+,-)
	  ]).

:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% has_property(+Object, -Type)
has_property(Object) :-
	subclass_of(Object, X),
	subclass_of(X, A),
	triple(A, B, suturo:'Fragility').

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Zum Testen: ein Bowl-Object erstellen
%% create_bowl(+Name)
create_bowl(Name):-
	kb_project(is_type(Name, suturo:'CerealBowl')).


test(Name, C) :-
	string_concat("das steht hier:", Name, C).

%hardcoded CerealBowl
has_propertyCerealBowl(Object) :-
	subclass_of(suturo:'CerealBowl', X),
	subclass_of(X, A),
	triple(A, B, suturo:'Fragility').

	