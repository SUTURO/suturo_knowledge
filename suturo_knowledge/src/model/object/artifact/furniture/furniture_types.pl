%% This file contains all the types of recognized furniture
:- module(furniture_types,
	  [
	      is_table/1,
	      is_shelf/1,
	      is_drawer/1
	  ]).

:- use_module('../../../types',
	      [
		  is_type/2
	      ]).


is_table(Table) ?+>
    is_type(Table, soma:'Table').

is_shelf(Shelf) ?+>
    is_type(Shelf, soma:'Shelf').

is_drawer(Drawer) ?+>
    is_type(Drawer, soma:'Drawer').
