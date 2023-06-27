# Rooms

A Room is a pose inside the semantic map that has a width and depth (aka y and x size)

Everything that is inside the rectangle described by the central position, the width, and the depth is inside the room.

A room has at least one entry point and at least one exit point.
Entry points can also be exit points.
If a room has no defined exit points, all entry points are also exit points.

The whole arena is a room.

## Main Predicates
```
room_entry(+Room, -Pose) is multi.
room_exit(+Room, -Pose) is multi.
is_inside_of(?Object, ?Room) is nondet.
```
Note: `is_inside_of` only works for objects created
(or updated via `object_pose(+,+)`)
after the room has been created.

## Types
```prolog
is_kitchen(?Room) is nondet.
is_living_room(?Room) is nondet.
is_bedroom(?Room) is nondet.
is_room(?Room) is nondet.
new_room_type(+Type) is det.
```
Warning: `new_room_type` fails if the type is an existing type in the knowledge base.


## Creating Rooms
```prolog
create_room(+RoomType, +Pose, +Depth, +Width, -IRI) is det.
create_room_entry(+Room,+Pose,-Entry) is det.
create_room_exit(+Room,+Pose,-Exit) is det.
create_room_entry_exit(+Room,+Pose,-Entry) is det.
```

## Technical details
- The shape of a room is a `soma:'BoxShape'` with a height of 4 meters (aka 2 meters underground and 2 meters aboveground).
