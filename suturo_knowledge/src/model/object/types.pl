%% This module contains predicates that are for object types.
:- module(types,
	  [
	      is_type(r,r)
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
