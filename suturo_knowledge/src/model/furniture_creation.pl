%% This module loads and creates furniture in the database.
:- module(furniture_creation,
	  [
	      create_table/2
	  ]).

create_table(Table, [Depth, Width, Height]) :-
    kb_project(is_table(Table)).
    %kb_project(object_dimensions(Table, Depth, Width, Height)).

is_table(Table) ?>
    has_type(Table, soma_home:'Table').

is_table(Table) +>
    new_iri(Table, soma_home:'Table'),
    has_type(Table, soma_home:'Table').
