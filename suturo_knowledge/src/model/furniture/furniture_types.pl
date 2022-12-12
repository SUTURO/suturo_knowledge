%% This file contains all the types of recognized furniture
:- module(furniture_types,
	  [
	      is_table/1,
	      is_drawer/1
	  ]).

:- use_module('../types',
	      [
		  is_type/2
	      ]).


is_table(Table) ?+>
    is_type(Table, soma:'Table').

is_drawer(Object) ?+>
    is_type(Object, soma:'Drawer').
