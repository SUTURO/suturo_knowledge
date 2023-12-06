%% The whereTo module reads the String of a location and gives back the pose.

%:- use_module(library('rosprolog')).
%:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).

:- module(fav_drink,
	  [
        fav_drink(+, -),
		is_known(+, -),
		save_me(+, +)
	  ]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Query: What is the favourite drink of person X?

%% fav_drink(+Name, -Drink)
fav_drink(Name, Drink) :-
	Drink = "Coffee".

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (2) Query: Is person X already known to us?

% has_type(Object, Type)
% kb_project(is_type(Object, Type), Scope)

%% is_known(+Name, -Bool)
is_known(Name, Bool) :-
	has_type(Name, Customer),
	kb_project(is_type(Name, Customer), Scope).

% was macht scope?

%save_me(+Name)
save_me(Name) :-
kb_project(triple(Name, suturo:isCustomer, Customer), Scope),
kb_project(is_type(Name, Person), Scope).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Query: Save the name X and favourite drink Y

% kb_project(triple(Object, suturo:hasConfidenceValue, ConfidenceValue), Scope)
% kb_project(is_type(Object, Type), Scope)
%% save_me_and_drink(+Name, +Drink)
save_me_and_drink(Name, Drink) :-
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink), Scope), 
	kb_project(triple(Name, suturo:isCustomer, Customer), Scope),
	kb_project(is_type(Name, Person), Scope),
	kb_project(is_type(Drink, Drink), Scope).



	




	