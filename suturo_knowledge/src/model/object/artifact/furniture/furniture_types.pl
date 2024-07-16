%% This file contains all the types of recognized furniture
:- module(furniture_types,
	  [
	      is_kitchen_table(r),
	      is_dining_table(r),
	      is_table(r),
	      is_shelf(r),
	      is_drawer(r),
	      is_shelf_layer(r),
		  is_trash_can(r),
		  is_coathanger(r),
		  is_dinner_table(r),
		  is_couch(r)
	  ]).

:- use_module(library('model/object/types'),
	      [
		  is_type/2
	      ]).


%% is_table(+Table) is det.
%
% Wrapper around is_type that also creates an iri on projection if Table is a variable.
%
% @param Table The table to check.
%
is_table(Table) ?+>
    is_type(Table, soma:'Table').

is_kitchen_table(Table) ?+>
    is_type(Table, suturo:'KitchenTable').
is_dining_table(Table) ?+>
    is_type(Table, suturo:'DiningTable').

%% is_shelf(+Shelf) is det.
%
% Wrapper around is_type that also creates an iri on projection if Shelf is a variable.
%
% @param Shelf The shelf to check.
%
is_shelf(Shelf) ?+>
    is_type(Shelf, suturo:'Shelf').


%% is_drawer(+Drawer) is det.
%
% Wrapper around is_type that also creates an iri on projection if Drawer is a variable.
%
% @param Drawer The Drawer to check.
%
is_drawer(Drawer) ?+>
    is_type(Drawer, soma:'Drawer').
    
%% is_shelf_layer(+Shelf) is det.
%
% Wrapper around is_type that also creates an iri on projection if Shelf is a variable.
%
% @param Shelf The shelf to check.
%
is_shelf_layer(ShelfLayer) ?+>
    is_type(ShelfLayer, suturo:'ShelfLayer').

is_trash_can(TrashCan) ?+>
    is_type(TrashCan, suturo:'TrashCan').

is_coathanger(CoatHanger) ?+>
    is_type(CoatHanger, suturo:'CoatHanger').

is_dinner_table(DinnerTable) ?+>
    is_type(DinnerTable, suturo:'DinnerTable').

is_hallway_cabinet(HallwayCabinet) ?+>
    is_type(HallwayCabinet, suturo:'HallwayCabinet').

is_tv_table(TvTable) ?+>
    is_type(TvTable, suturo:'TvTable').

is_kitchen_counter(KitchenCounter) ?+>
    is_type(KitchenCounter, suturo:'KitchenCounter').

is_coffee_table(CoffeeTable) ?+>
    is_type(CoffeeTable, suturo:'CoffeeTable').

is_desk(Desk) ?+>
    is_type(Desk, suturo:'Desk').

is_couch(Couch) ?+>
	is_type(Couch, suturo:'Couch').
