%% The whereTo module reads the String of a location and gives back the pose.

%:- use_module(library('rosprolog')).

:- module(fav_drink,
	  [
        fav_drink(+, -),
		is_customer(+),
		save_me(+),
		save_me_and_drink(+, +)
	  ]).

:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Query: What is the favourite drink of person X?

%% fav_drink(+Name, -Drink)
fav_drink(Name, Drink) :-
 	kb_call((has_type(Name, Class),
 		holds(Name, suturo:hasFavouriteDrink, Drink))),
	% if one exists, use only that
	!.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (2) Query: Is person X already known to us?

% has_type(Object, Type)
% kb_project(is_type(Object, Type), Scope)

%% is_customer(+Name)
is_customer(Name) :-
	has_type(Name, suturo:'Customer').
	

%% save_me(+Name)
save_me(Name) :-
	kb_project(is_type(Name, suturo:'Customer')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Query: Save the name X and favourite drink Y

% kb_project(triple(Object, suturo:hasConfidenceValue, ConfidenceValue), Scope)
% kb_project(is_type(Object, Type), Scope)

%% save_me_and_drink(+Name, +Drink)
save_me_and_drink(Name, 'Coffee') :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Coffee')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_drink(Name, 'RaspberryJuice') :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'RaspberryJuice')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_drink(Name, 'Milk') :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Milk')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_drink(Name, 'Tea') :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Tea')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).


	