:- module(reset, [
			  reset_user_data/0
		  ]).

%% reset_user_data is det.
%
% drop the user graph (containing objects and data assigned at runtime)
% and reinitialize the furnitures from the semantic map.
reset_user_data :-
	drop_graph(user),
	init_furnitures.
