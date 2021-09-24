:- module(misplaced,
    [
        misplaced_objects_at_predefined_location/3,
        is_misplaced/1
    ]).


:- use_module(library('locations/actual_locations'), [object_at_location/4]).
:-use_module(library('locations/predefined_locations'), [object_at_predefined_location/3]).
:- use_module(library('model/objects/object_info'), [is_suturo_object/1]).



%% misplaced_objects_at_predefined_location(?Objects, ?RoomType, ?FurnitureType) is det
%
% returns all objects, which are misplaced and have the same predefined location
%
% @param Objects, list of object instances
% @param RoomType, subclass of class hsr_rooms:'Room'
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
misplaced_objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_at_predefined_location(Object, RoomType, FurnitureType),
        is_misplaced(Object)
    ), 
    Objects).



%% is_misplaced(?Object) is nondet
%
% True if the given object is misplaced meaning actual and predefined location
% do not match
% 
% @param Object, object instance
%
is_misplaced(Object) :-
    object_at_location(Object, Room, Furniture, _),
    writeln("Current Location"),
    writeln(Furniture),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    writeln("Predefined Location"),
    writeln(FurnitureType),
    not (has_type(Room, RoomType), has_type(Furniture, FurnitureType)).