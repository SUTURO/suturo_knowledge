:- module(room_relations,
          [ is_entry_to(r,r),
            is_exit_from(r,r),
            room_entry(r,-),
            room_exit(r,-),
            is_inside_of(r,r)
          ]).

is_entry_to(Location, Room) ?+>
    triple(Location, suturo:isEntryTo, Room).

is_exit_from(Location, Room) ?+>
    triple(Location, suturo:isExitFrom, Room).

room_entry(Room, Pose) ?>
    is_entry_to(Entry, Room),
    is_at(Entry, Pose).

room_exit(Room, Pose) ?>
    is_exit_from(Exit, Room),
    is_at(Exit, Pose).

is_inside_of(Object, Room) ?+>
    triple(Object, soma:isInsideOf, Room).
