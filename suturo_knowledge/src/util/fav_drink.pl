%% The fav_drink module enables to ask for a favourite drink, 
%	if a person is known to us or to save the information name and favourite drink.

:- module(fav_drink,
	  [
        fav_drink(+, -),
		is_customer(+),
		save_me(+, +),
		has_id(+, -),
		has_name(+, -),
		ask_both(?, ?),
		save_me_and_coffee(+, +),
		save_me_and_raspberryjuice(+, +),
		save_me_and_milk(+, +),
		save_me_and_tea(+, +),
		save_me_and_water(+, +)
	  ]).

:- load_owl('package://suturo_knowledge/owl/suturo.owl', [namespace(suturo, 'http://www.ease-crc.org/ont/SUTURO.owl#')]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (1) Query: What is the favourite drink of person X?

%% fav_drink(+Name, -Drink)
fav_drink(Name, Drink):-
 	kb_call(holds(Name, suturo:hasFavouriteDrink, Drink)),
	% if one exists, use only that
	!.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (2) Query: Is person X already known to us?

%% is_customer(+Name)
is_customer(Name):-
	has_type(Name, suturo:'Customer').

%% save_me(+Name,+ID)
save_me(Name, ID):-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)).

%% has_id(+Name,-ID)
has_id(Name, ID):-
	kb_call(is_type(Name, suturo:'Customer')),
	kb_call(holds(Name, suturo:hasCustomerID, ID)),
	% if one exists, use only that
	!.

%% has_name(+ID, -Name)
has_name(ID, Name):-
	kb_call(holds(Name, suturo:hasCustomerID, ID)),
	% if one exists, use only that
	!.

%% ask_both(?Name,?ID)
ask_both(Name, ID):-
	kb_call(holds(Name, suturo:hasCustomerID, ID)),
	% if one exists, use only that
	!.

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Query: Save the name X and favourite drink Y

%% save_me_and_coffee(+Name, +ID)
save_me_and_coffee(Name, ID) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)),
	kb_project(is_type(Drink, suturo:'Coffee')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

%% save_me_and_raspberryjuice(+Name, +ID)
save_me_and_raspberryjuice(Name, ID) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)),
	kb_project(is_type(Drink, suturo:'RaspberryJuice')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

%% save_me_and_milk(+Name, +ID)
save_me_and_milk(Name, ID) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)),
	kb_project(is_type(Drink, suturo:'Milk')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

%% save_me_and_tea(+Name, +ID)
save_me_and_tea(Name, ID) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)),
	kb_project(is_type(Drink, suturo:'Tea')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

%% save_me_and_water(+Name, +ID)
save_me_and_water(Name, ID) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(triple(Name, suturo:hasCustomerID, ID)),
	kb_project(is_type(Drink, suturo:'Water')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).
