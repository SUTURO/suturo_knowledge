# Planning API

## Readers guide
Each section contains
1. a short description of a possible use case
2. the definition of the predicate
3. a codeblock with multiple examples, each one separated by 2 newlines

The examples are in the following format:
1. `?- ` followed by a query
2. The variable instantiations that Prolog found.

Meaning of the symbols in front of the parameter:
These symbols are the same as on <https://www.swi-prolog.org/pldoc/man?section=modes>.
In short:
- `?` means this can be specified (an input) or be a variable (an output)
- `+` means this is only input (it has to be a concrete value)
- `-` means this is output (only putting a variable here makes sense for planning)

The definitions of what it is is also taken from there.
In short:
- `det` means it will always succeed once
- `semidet` means it will succeed once or fail
- `nondet` may succeed any number of times (this means `cut:lazy-cdr` might be needed to get all values)
- `multi` succeeds at least once

## Data types

### Object
An iri, for example `'http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_MBOLQEWJ'` that denotes an object.

### PoseStamped
A list of Frame, Position and Rotation.
Position and Rotation are each a list of length 3 and 4.

`[Frame, [X, Y, Z], [RX, RY, RZ, RW]]`

## Getting the object for a link name
This is useful for getting data about specific environment furniture, for example the tall table.

```prolog
has_urdf_name(?Object, ?URDFName) is nondet.
```

Examples:
```
?- has_urdf_name(Object, URDFName).
Object: http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_fridge_door_handle,
URDFName: iai_fridge_door_handle ;

Object: http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_oven_area_area,
URDFName: oven_area_area ;

Object: http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_LXEIYGPZ,
URDFName: long_table:table:table_front_edge_center ;

Object: http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_MBOLQEWJ,
URDFName: tall_table:table:table_front_edge_center.


% Notice that there are single quotes around the urdf because it contains non [a-z_] characters.
?- has_urdf_name(Object, 'tall_table:table:table_front_edge_center').
Object: http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_MBOLQEWJ.


?- furniture_creation:has_urdf_name('http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_MBOLQEWJ', URDFName).
URDFName: tall_table:table:table_front_edge_center.
```

## Getting the TF Frame

```prolog
has_tf_name(+Object, -TFName) is semidet.
```

Examples:
```
?- has_tf_name('tall_table:table:table_front_edge_center',TFName).
TFName: iai_kitchen/tall_table:table:table_front_edge_center.


?- has_tf_name('http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_LTKIUPNG', TFName).
TFName: iai_kitchen/tall_table:table:table_front_edge_center.
```

## Getting the pose of an object

```prolog
object_pose(+Object, -PoseStamped) is semidet.
```

Examples:
```
?- object_pose('http://www.ease-crc.org/ont/SOMA-HOME.owl#Table_LTKIUPNG', Pose)
Pose: ['iai_kitchen/tall_table:table:table_front_edge_center', [0,0,0], [0,0,0,1]]
```

## Getting important poses of furnitures
Note: not implemented yet

Gets a position relative to a furniture based on the type of relation.

Valid `Type`s are `perceive` and `interact`. This list might be expanded later.

```prolog
furniture_rel_pose(+Furniture, +Type, -PoseStamped) is semidet.
```
