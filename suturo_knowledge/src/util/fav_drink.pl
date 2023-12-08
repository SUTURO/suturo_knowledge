%% The whereTo module reads the String of a location and gives back the pose.

%:- use_module(library('rosprolog')).

:- module(fav_drink,
	  [
        fav_drink(+, -),
		is_customer(+),
		save_me(+),
		save_me_and_coffee(+),
		save_me_and_raspberryjuice(+),
		save_me_and_milk(+),
		save_me_and_tea(+)
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

%% is_customer(+Name)
is_customer(Name) :-
	has_type(Name, suturo:'Customer').
	

%% save_me(+Name)
save_me(Name) :-
	kb_project(is_type(Name, suturo:'Customer')).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% (3) Query: Save the name X and favourite drink Y


%% save_me_and_drink(+Name)
save_me_and_coffee(Name) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Coffee')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_raspberryjuice(Name) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'RaspberryJuice')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_milk(Name) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Milk')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).

save_me_and_tea(Name) :-
	kb_project(is_type(Name, suturo:'Customer')),
	kb_project(is_type(Drink, suturo:'Tea')),
	kb_project(triple(Name,suturo:hasFavouriteDrink, Drink)).


	