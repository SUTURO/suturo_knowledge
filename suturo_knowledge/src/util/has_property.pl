%% The has_property module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(has_property,
	  [
		create_bowl(+),
        has_property(+, -)
	  ]).

:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%% has_property(+Object, -Type)
has_property(Name, Type):-
	holds(Name, suturo:'Fragility', Type).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Zum Testen: ein Bowl-Object erstellen
%% create_bowl(+Name)
create_bowl(Name):-
	kb_project(is_type(Name, suturo:'Bowl')).



 	
	