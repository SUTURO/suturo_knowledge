%% This module contains predicates that are for object types.
:- module(types,
	  [
	      is_type(r,r)
	  ]).

%% is_type(+Obj, +Type) is semidet.
%
% Wrapper around has_type that also creates an iri on projection if Obj is a variable.
%
% @param Obj The object to check the type of.
% @param Type The type to check.
%
is_type(Obj, Type) ?>
    has_type(Obj, Type).

%% is_type(+Obj, +Type) is semidet.
%
% Wrapper around has_type that also creates an iri on projection if Obj is a variable.
%
% @param Obj The object to check the type of.
% @param Type The type to check.
%
is_type(Obj, Type) +>
    ( var(Obj) ->
      new_iri(Obj, Type);
      true),
    has_type(Obj, Type).
