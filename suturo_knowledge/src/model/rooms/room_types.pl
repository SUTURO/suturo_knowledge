:- module(room_types,
          [ is_kitchen(r),
            is_living_room(r),
            is_bedroom(r),
            is_room(r),
            new_room_type(r)]).

is_kitchen(Room) ?+>
    is_type(Room, soma:'Kitchen').

is_living_room(Room) ?+>
    is_type(Room, suturo:'LivingRoom').

is_bedroom(Room) ?+>
    is_type(Room, suturo:'Bedroom').

is_room(Room) ?>
    has_type(Room, soma:'Room').

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
