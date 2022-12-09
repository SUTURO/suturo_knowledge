%% This file contains all the types of recognized furniture
:- module(furniture_types,
	  [
	      is_table/1,
	      is_type/2
	  ]).

%%
%
% Wrapper around has_type that also creates an iri on projection
is_type(Obj, Type) ?>
    has_type(Obj, Type).

is_type(Obj, Type) +>
    ( var(Obj) ->
      new_iri(Obj, Type);
      true),
    has_type(Obj, Type).


is_table(Table) ?+>
    is_type(Table, soma_home:'Table').
