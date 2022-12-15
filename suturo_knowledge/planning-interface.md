# Planning API

## Overview
- [Readers guide](#readers-guide)
- [Data types](#data-types)
- [Utils](#utils)
- [Objects](#objects)

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
An iri, for example `'http://www.ease-crc.org/ont/SOMA.owl#Table_MBOLQEWJ'` that denotes an individual object.

### PoseStamped
A list of Frame, Position and Rotation.
Position and Rotation are each a list of length 3 and 4.

`[Frame, [X, Y, Z], [RX, RY, RZ, RW]]`

## Utils

### Getting the object for a link name
This is useful for getting data about specific environment furniture, for example the tall table.

```prolog
has_urdf_name(?Object, ?URDFName) is nondet.
```

Example:
```prolog
?- has_urdf_name(Object, URDFName).
Object: http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_fridge_door_handle,
URDFName: iai_fridge_door_handle ;

Object: http://knowrob.org/kb/IAI-kitchen.owl#iai_kitchen_oven_area_area,
URDFName: oven_area_area ;

Object: http://www.ease-crc.org/ont/SOMA.owl#Table_LXEIYGPZ,
URDFName: long_table:table:table_front_edge_center ;

Object: http://www.ease-crc.org/ont/SOMA.owl#Table_MBOLQEWJ,
URDFName: tall_table:table:table_front_edge_center.


% Notice that there are single quotes around the urdf because it contains non [a-z_] characters.
?- has_urdf_name(Object, 'tall_table:table:table_front_edge_center').
Object: http://www.ease-crc.org/ont/SOMA.owl#Table_MBOLQEWJ.


?- furniture_creation:has_urdf_name('http://www.ease-crc.org/ont/SOMA.owl#Table_MBOLQEWJ', URDFName).
URDFName: tall_table:table:table_front_edge_center.
```

### Getting the TF Frame

```prolog
has_tf_name(+Object, -TFName) is semidet.
```

Example:
```prolog
?- has_tf_name('tall_table:table:table_front_edge_center',TFName).
TFName: iai_kitchen/tall_table:table:table_front_edge_center.


?- has_tf_name('http://www.ease-crc.org/ont/SOMA.owl#Table_LTKIUPNG', TFName).
TFName: iai_kitchen/tall_table:table:table_front_edge_center.
```
## Objects
Any physical object that has a proper space region.

### Create an object

Create an object of a given Type at the given PoseStamped.

The `Type` has to be the full iri for now. The ability to use namespace short form in future is planned.
```prolog
create_object(-Object, +Type, +PoseStamped) is det.
```

Example:
```prolog
?- create_object(Object, 'http://www.ease-crc.org/ont/SOMA.owl#CerealBox', ['iai_kitchen/long_table:table:table_front_edge_center', [0,1,1], [0,0,0.70711,0.70711]]).
Object: http://www.ease-crc.org/ont/SOMA.owl#CerealBox_LTKIUPNG
```

### Getting the pose of an object

```prolog
object_pose(+Object, -PoseStamped) is semidet.
```

Example:
```prolog
?- object_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_LTKIUPNG', Pose)
Pose: ['iai_kitchen/tall_table:table:table_front_edge_center', [0,0,0], [0,0,0,1]]
```

### Getting important poses for the robot to interact with objects

Gets a position relative to the object based on the type of relation.

Valid `Type`s are 
 - `perceive`: (Optimal) Pose for the robot to position at to perceive the object
 - `interact`: (Optimal) Pose for the robot to position at to interact with the object (eg. grasp with the gripper)
 - `destination`: Returns the destination pose of an object. The destination pose is the pose where the object should be placed based on their predefined locations and current context. The destination might change over time.
 
 This list might be expanded later.

```prolog
object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
```

Example:
```prolog
?- object_rel_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_YTORLZXJ', perceive, Pose).
Pose: ['iai_kitchen/long_table:table:table_front_edge_center', [-0.7, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]].

?- object_rel_pose('http://www.ease-crc.org/ont/SOMA.owl#CerealBox_JDHUPSME', destination, Pose).
Pose: ['iai_kitchen/shelf:shelf:shelf_base_center', [0.0, -0.1, 0.51], [0.0, 0.0, 0.0, 1.0]].
```