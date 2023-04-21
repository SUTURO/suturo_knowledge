# Poses relative to objects
This folder is about getting poses that are relative to objects, for example a table.

## IMPORTANT - Approach Direction
For multiple position types to `object_rel_pose/4` the option `direction(Dir)` is supported. Currently this is implemented for the types `perceive` and `place`.
`Dir` should be one of `['-x', '+x', '-y', '+y']` and describes the direction where the robot should be relative to the object.

For example if the robots x coordinate should be smaller than the x coordinate of the object, the direction `'-x'` should be used. (This is also the default direction).

The directions are relative to the frame of the **object**.

## Perceive Pose
The preceive pose is a pose in front of the object, that is optimized for perceiving objects on/in the object.

Currently this is implemented by return a pose a fixed distance in front of the object. For Shelfs this is currently 1.06 meters, for everything else 0.67 meters.

## Place Pose
The place pose is a pose on top of the obejct suitable for placing an object there.

Currently this only works for tables. It is implemented by segmenting the length perpendicular to the approach direction into MaxIndex segmentes and return the center of the segment Index. Index is 1-based, so if Index is 0, the position is on the side of the table in the air.

MaxIndex and Index are required options, an for example a call can look like this:
```prolog
?- object_rel_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_NXHPQBIU', place, [direction('-x'),index(2),maxindex(5)], Pose)
Pose: ['iai_kitchen/left_table:table:table_front_edge_center', [-0.27499999999999997, 0.28, 0.0], [0, 0, 0, 1]].
```
