:- module(predefined_locations,
    [
        object_at_predefined_location/3,
        surfaces_at_predefined_location/3,
        surface_at_predefined_location/3
    ]).


:- use_module(library('model/environment/furnitures'), [has_surface/2]).
 
:- use_module(library('locations/actual_locations'), [furniture_in_room/2]).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% predefined object locations %%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% object_at_predefined_location(?Object, ?RoomType, ?FurnitureType) is nondet
%
% True if the predefined location of the given object is 
% defined by the classes RoomType and FurnitureType
%
% @param Object, object instance
% @param RoomType, subclass of class hsr_rooms:'Room'
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
object_at_predefined_location(Object, RoomType, FurnitureType) :-
    object_in_predefined_room(Object, RoomType),
    object_on_predefined_furniture(Object, FurnitureType).


%% objects_at_predefined_location(?Object, ?RoomType, ?FurnitureType) is det
%
% Returns all objects having a predefined location defined as 
% class RoomType and class FurnitureType
%
% @param Objects, list of object instances
% @param RoomType, subclass of class hsr_rooms:'Room'
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object, object_at_predefined_location(Object, RoomType, FurnitureType), Objects).


%% objects_at_same_predefined_location(?Object, ?Objects) is det
%
% Returns all objects having the same predfined as the given object
%
% @param Object, object instance
% @param Objects, list of object instances
%
objects_at_same_predefined_location(Object, Objects) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    objects_at_predefined_location(Objects, RoomType, FurnitureType).


%% object_in_predefined_room(?Object, ?RoomType) is nondet
%
% True if the given object has a predefined room of class RoomType
%
% @param Object, object instance
% @param RoomType, subclass of class hsr_rooms:'Room'
%
object_in_predefined_room(Object, RoomType) :-
    has_predefined_location(Object, Location),
    triple(Location, knowrob:'isInsideOf', RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').


%% object_on_predefined_furniture(?Object, ?FurnitureType) is nondet
%
% True if the given object has a predefined furniture of class FurnitureType
%
% @param Object, object instance
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
object_on_predefined_furniture(Object, FurnitureType) :-
    has_predefined_location(Object, Location),
    (triple(Location, knowrob:'isOntopOf', FurnitureType);
    triple(Location, knowrob:'isInsideOf', FurnitureType)),
    subclass_of(FurnitureType, soma:'DesignedFurniture').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% predefined surface locations %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% surface_at_predefined_location(?Surface, ?RoomType, ?FurnitureType) is nondet
%
% True if the location of Surface matches the predefined location defined by classes
% RoomType and FurnitureType
%
% @param Surface, surface instance
% @param RoomType, subclass of class hsr_rooms:'Room'
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
surface_at_predefined_location(Surface, RoomType, FurnitureType) :-
    has_surface(Furniture, Surface),
    has_type(Furniture, FurnitureType),
    subclass_of(FurnitureType, soma:'DesignedFurniture'),
    furniture_in_room(Furniture, Room),
    has_type(Room, RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').


%% surfaces_at_predefined_location(?Surfaces, ?RoomType, ?FurnitureType) is det
%
% Returns all surfaces of which the location matches the predefined location
% defined by classes RoomType, FurnitureType
%
% @param Surface, list of surface instances
% @param RoomType, subclass of class hsr_rooms:'Room'
% @param FurnitureType, subclass of class soma:'DesignedFurniture'
%
surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType) :-
    findall(Surface, surface_at_predefined_location(Surface, RoomType, FurnitureType), Surfaces).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% general %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% has_predefined_location(?Object, ?Location) is nondet
%
% True if a predfined location is defined for Object
%
% @param Object, object instance
% @param Location, instnace of type soma:'Location'
%
has_predefined_location(Object, Location) :-
    once((
        has_type(Object, ObjectType),
        transitive(subclass_of(ObjectType, SupportedType)),
        triple(SupportedType, hsr_rooms:'hasPredefinedLocation', Location)
    )).