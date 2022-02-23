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


object_at_predefined_location(Object, RoomType, FurnitureType) :-
    object_in_predefined_room(Object, RoomType),
    object_on_predefined_furniture(Object, FurnitureType).


objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object, object_at_predefined_location(Object, RoomType, FurnitureType), Objects).


objects_at_same_predefined_location(Object, Objects) :-
    object_at_predefined_location(Object, RoomType, FurnitureType),
    objects_at_predefined_location(Objects, RoomType, FurnitureType).


object_in_predefined_room(Object, RoomType) :-
    has_predefined_location(Object, Location),
    triple(Location, knowrob:'isInsideOf', RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').


object_on_predefined_furniture(Object, FurnitureType) :-
    has_predefined_location(Object, Location),
    triple(Location, knowrob:'isOntopOf', FurnitureType).
    %triple(Location, knowrob:'isInsideOf', FurnitureType)).
    %subclass_of(FurnitureType, soma:'DesignedFurniture').


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%% predefined surface locations %%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

surface_at_predefined_location(Surface, RoomType, FurnitureType) :-
    has_surface(Furniture, Surface),
    has_type(Furniture, FurnitureType),
    subclass_of(FurnitureType, soma:'DesignedFurniture'),
    furniture_in_room(Furniture, Room),
    has_type(Room, RoomType),
    subclass_of(RoomType, hsr_rooms:'Room').


surfaces_at_predefined_location(Surfaces, RoomType, FurnitureType) :-
    findall(Surface, surface_at_predefined_location(Surface, RoomType, FurnitureType), Surfaces).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%% general %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

has_predefined_location(Object, Location) :-
    once((
        has_type(Object, ObjectType),
        transitive(subclass_of(ObjectType, SupportedType)),
        triple(SupportedType, hsr_rooms:'hasPredefinedLocation', Location)
    )).
