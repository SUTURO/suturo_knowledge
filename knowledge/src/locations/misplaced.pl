:- module(misplaced,
    [
        misplaced_objects_at_predefined_location/3,
        is_misplaced/1
    ]).



misplaced_objects_at_predefined_location(Objects, RoomType, FurnitureType) :-
    findall(Object,
    (
        is_suturo_object(Object),
        object_at_predefined_location(Object, RoomType, FurnitureType),
        is_misplaced(Object)
    ), 
    Objects).


is_misplaced(Object) :-
    object_at_location(Object, Room, Furniture, _),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    not (has_type(Room, RoomType), has_type(Furniture, FurnitureType)).