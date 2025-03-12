:- module(room_types,
          [ is_kitchen(r),
            is_living_room(r),
            is_bedroom(r),
            is_room(r),
            is_room_2(r),
            is_study_room(r),
            is_dining_room(r),
            is_pantry(r),
            is_office(r),
            is_corridor(r),
            new_room_type(r)]).

is_kitchen(Room) ?+>
    is_type(Room, soma:'Kitchen').

is_living_room(Room) ?+>
    is_type(Room, suturo:'LivingRoom').

is_bedroom(Room) ?+>
    is_type(Room, suturo:'Bedroom').

is_arena(Room) ?+>
    is_type(Room, suturo:'Arena').

is_study_room(Room) ?+>
    is_type(Room, suturo:'StudyRoom').

is_room(Room) ?>
    has_type(Room, soma:'Room').

is_room_2(Room) ?>
    has_type(Room, soma:'Room'),
    \+ has_type(Room, 'http://www.ease-crc.org/ont/SUTURO.owl#Arena').
    
is_dining_room(Room) ?>
    has_type(Room, suturo:'DiningRoom').

is_pantry(Room) ?>
    has_type(Room, suturo:'Pantry').

is_office(Room) ?>
    has_type(Room, suturo:'Office').

is_corridor(Room) ?>
    has_type(Room, suturo:'Corridor').

%% new_room_type(+Type) is semidet.
%
% If Type is already a type (aka it has a supertype),
% fail with an error, otherwise set it up as a subclass of soma:'Room'.
new_room_type(Type) :-
    kb_call(subclass_of(Type,SuperClass)),
    ros_error('new_room_type: ~w already has a superclass, ~w', [Type,SuperClass]),
    !,
    fail.
new_room_type(Type) :-
    kb_project(subclass_of(Type,soma:'Room')).
