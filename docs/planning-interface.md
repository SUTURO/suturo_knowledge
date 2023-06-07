# Planning API Documentation

## Readers guide

Each code documentation section contains:

1. A short description of a possible use case
2. The definition of the predicate
3. A codeblock with multiple examples, each one separated by 2 newlines

The examples are in the following format:

1. `?- ` followed by a query
2. The variable instantiations that Prolog found.

Meaning of the symbols in front of the parameter:

- `?` means this can be specified (an input) or be a variable (an output)
- `+` means this is only input (it has to be a concrete value)
- `-` means this is output (only putting a variable here makes sense for planning)

Meaning of the predicate behavior:  

- `det` Succeeds exactly once without a choice point
- `semidet` Fails or Succeeds exactly once without a choice-point
- `nondet` May succeed any number of times (this means `cut:lazy-cdr` might be needed to get all values)
- `multi` As `nondet`, but succeeds at least one time

These definitions are the same as on <https://www.swi-prolog.org/pldoc/man?section=modes>.

## High-Level Concepts

In the following the high-level ideas for the exchange between Planning and Knowledge are described for the different challenges.

To successfully utilize the Knowledge-Planning API, the following 4 predicates are important:

- `create_object(-Object, +Type, +PoseStamped, +Options)`: Used to instantiate/create new, perceived objects
- `object_rel_pose(+Object, +Type, +Options, -PoseStamped)`: Returns important poses for a given object (e.g. from where to perceive it or where to place it)
- `predefined_origin_location(+Type, -Location)`: Returns the predefined location for a given object type (e.g. the predefined location of bowls is on inside of Shelf_FIFFJO)
- `next_object(-Object)`: Returns the next best object to be processed/picked for a given challenge

### Serve Breakfast
[![](https://mermaid.ink/img/pako:eNqNVF1v2jAU_StXfgHUQAMtjeqHPmxdpZVVRe3TVqrIcS7UI9hZbPqxiv--6yRAoNVEJOzE99zj43vMfWfSpMg4s_hniVripRKzQiwmGiAXhVNS5UI7GCMIS2MhMXfK6A_xMpwJrZWe7QdHPjjS5iXDdIY-Ou5eXIw53JhnhDuTGAdKOwPuibYpUBN8rpx8Qg0vfkiNKUBZMDnqdfqIQ15gilOlMY1NoWZKx5mRwstrW7MQvPWFtmwF0B1vgD9qQAdouxm6Bgesk8FMIaFM67cadSupHyk4fBUJLbj45_X3q-urq8uGNJP8RuniArM4NxbbrX0syWrlvpzqGen96LYsq_ViCX_vxCLHdKuyRoIn8_p8pWRFSaWzTpB1O3K3JBweWkqouK7ocZ3G9-Y4ERZjidphQYIewl5IYsJeP4CwN-w_rpe2Q78XPj7uu0mC17p2RG9wyKtr5ANllb1-lW6OUwJxfYwamZZQEDoF5ewOIRVbFigcxlXN293bcg5g5w4cNSqyLXenzq3oPytkRcbB0xxk9A6wdDkT8gCLU7RO6eoCNm-id9qrO9BdJ5IMeWP8r69nh_g6VnK-kVF6UB6JnADhysCn2tv088FSRafBd4c51oki8XfGOmoqlibfM_TMs9ZlbfzD17zHA5hSN_DZhoaihlpo29wYHYBEsjRLzGsAC5XNcyHnHRawBRYLoVJqde9ey4RR8gInjNMr7SGWmZuwiV4RVCyduX_TknFXLDFgyzylO1J3RsanIrOb1W-pcqbYLGL5eVP11LK1Box64C9jton0zfg7e2X85Kx3etZ4ooC9Md4fnvcGYRRFg5CmwfB0FbC_JQE5Uz3D8Dw6ifpRtPoHFBTn2Q?type=png)](https://mermaid.live/edit#pako:eNqNVF1v2jAU_StXfgHUQAMtjeqHPmxdpZVVRe3TVqrIcS7UI9hZbPqxiv--6yRAoNVEJOzE99zj43vMfWfSpMg4s_hniVripRKzQiwmGiAXhVNS5UI7GCMIS2MhMXfK6A_xMpwJrZWe7QdHPjjS5iXDdIY-Ou5eXIw53JhnhDuTGAdKOwPuibYpUBN8rpx8Qg0vfkiNKUBZMDnqdfqIQ15gilOlMY1NoWZKx5mRwstrW7MQvPWFtmwF0B1vgD9qQAdouxm6Bgesk8FMIaFM67cadSupHyk4fBUJLbj45_X3q-urq8uGNJP8RuniArM4NxbbrX0syWrlvpzqGen96LYsq_ViCX_vxCLHdKuyRoIn8_p8pWRFSaWzTpB1O3K3JBweWkqouK7ocZ3G9-Y4ERZjidphQYIewl5IYsJeP4CwN-w_rpe2Q78XPj7uu0mC17p2RG9wyKtr5ANllb1-lW6OUwJxfYwamZZQEDoF5ewOIRVbFigcxlXN293bcg5g5w4cNSqyLXenzq3oPytkRcbB0xxk9A6wdDkT8gCLU7RO6eoCNm-id9qrO9BdJ5IMeWP8r69nh_g6VnK-kVF6UB6JnADhysCn2tv088FSRafBd4c51oki8XfGOmoqlibfM_TMs9ZlbfzD17zHA5hSN_DZhoaihlpo29wYHYBEsjRLzGsAC5XNcyHnHRawBRYLoVJqde9ey4RR8gInjNMr7SGWmZuwiV4RVCyduX_TknFXLDFgyzylO1J3RsanIrOb1W-pcqbYLGL5eVP11LK1Box64C9jton0zfg7e2X85Kx3etZ4ooC9Md4fnvcGYRRFg5CmwfB0FbC_JQE5Uz3D8Dw6ifpRtPoHFBTn2Q)

### Storing Groceries
[![](https://mermaid.ink/img/pako:eNrFVFFP2zAQ_isnv5SKNKRAicgDD4MxjYKogIcBRZHjXFuP1M4SF-gQ_31nJy1Z0NB4mJaHJL777nz3ffY9M6FTZBEr8ccClcAjyacFn48VQM4LI4XMuTIwQuAlvQuBuZFavfE7d8aVkmradg6tc6j0Y4bpFK131Ds4GEVwph8QLnSiDUhlNPACFYfHGSpItS5AlqBzVO0IQgqeSIUGuEodZG2wcSv8MAKdfEdh4gKzONclbnQOK1x8fXV-cXrz7aTjQSe3bckHpP_Nc9de6UFvRPhLw-c5pl2bcdirSmjYI7jtSC7je2kEFb1VFxG1vnHCS4wFKoMF7XEb-AHlD_y-B4E_6N-tTK-vvh_c3b3T9qpisE2tcRhVClkHPmCxNDNSg6gtZYqrWIfGupdTWRrQk5qm0tFpZigLl7hsECkK5AbjCrjRO3df4utqmaMHTVJoVZPYhQmJiFzM1gWn9VbvSXTFkwwrgb4cXbYEgj8L1OLK2DQfY0or230V-U942vyNp80P8aTwybTydm2fUzoP1gcJ2iKdw9pzKe6b57YKieAQqcTsk36Kr0--Hp8cHx-9e1_eoJ0gGRf_57rs_c11GVHrpA9f02H1cjWDpIVxKqdEl1Tc1g-ZFnw11uocF5hjjeSJPVKlodFXwkZDhy0qydhp1abN9_0GS7TqdpnH5ljMuUxp2j7bjcaMYuc4ZhH9pjjhi8yM2Vi9EJQvjL5cKsEiUyzQY4s8pWNVD2cWTXhWrq2fU2l0sTaiW55VY91Nd4_RGL7R-jWQ1ix6Zk8s2tnzd_caT-ixJYv6g31_OwjDcDugz_Zg98VjP10Corp6BsF-uBP2w_DlF-_ODPE?type=png)](https://mermaid.live/edit#pako:eNrFVFFP2zAQ_isnv5SKNKRAicgDD4MxjYKogIcBRZHjXFuP1M4SF-gQ_31nJy1Z0NB4mJaHJL777nz3ffY9M6FTZBEr8ccClcAjyacFn48VQM4LI4XMuTIwQuAlvQuBuZFavfE7d8aVkmradg6tc6j0Y4bpFK131Ds4GEVwph8QLnSiDUhlNPACFYfHGSpItS5AlqBzVO0IQgqeSIUGuEodZG2wcSv8MAKdfEdh4gKzONclbnQOK1x8fXV-cXrz7aTjQSe3bckHpP_Nc9de6UFvRPhLw-c5pl2bcdirSmjYI7jtSC7je2kEFb1VFxG1vnHCS4wFKoMF7XEb-AHlD_y-B4E_6N-tTK-vvh_c3b3T9qpisE2tcRhVClkHPmCxNDNSg6gtZYqrWIfGupdTWRrQk5qm0tFpZigLl7hsECkK5AbjCrjRO3df4utqmaMHTVJoVZPYhQmJiFzM1gWn9VbvSXTFkwwrgb4cXbYEgj8L1OLK2DQfY0or230V-U942vyNp80P8aTwybTydm2fUzoP1gcJ2iKdw9pzKe6b57YKieAQqcTsk36Kr0--Hp8cHx-9e1_eoJ0gGRf_57rs_c11GVHrpA9f02H1cjWDpIVxKqdEl1Tc1g-ZFnw11uocF5hjjeSJPVKlodFXwkZDhy0qydhp1abN9_0GS7TqdpnH5ljMuUxp2j7bjcaMYuc4ZhH9pjjhi8yM2Vi9EJQvjL5cKsEiUyzQY4s8pWNVD2cWTXhWrq2fU2l0sTaiW55VY91Nd4_RGL7R-jWQ1ix6Zk8s2tnzd_caT-ixJYv6g31_OwjDcDugz_Zg98VjP10Corp6BsF-uBP2w_DlF-_ODPE)

### Clean the Table
[![](https://mermaid.ink/img/pako:eNqNVN9P2zAQ_lcsvxS0tKRAB_MDT12lESEq4GWjKHLsa-uR2pntAgXxv-_sJG1gE8IPtnO__N3dl3uhwkigjDr4swYtYKz4wvLVTBNSceuVUBXXnkyBcIe7FVB5ZfQ_-qguudZKL94rs6DMtHksQS4gaKf9s7MpIxfmAciVKYwnSntD_BKfsaDR_F55sQRNHsMmjbFEOWIq0K17xogpfoPwuYUyr4yDvd5YhedveFFC_vP8x-R8Mhn3EtKrAm71AHj_chnxu4T0p-hz7fmqArlP8PUFeNJakhCQmHmEJHihNASMznOsUYCQ9esUOkEYue0prvIG-oEPOFhnzwvuIBegPViEcpsOUoSRDoYJSQej4V0r2m3DQXp3975gCFXGREmM-hbz1hhY3a6ggAewG78MLkbHlLoBogu0-TQ-sqmuI1zL4KJsjO869RcWuIe8NtzrX8YTS3yzqQCPTml2dd8nc-wlcLHc4m6f6kTW8OTfxd22KOhIAc43bkFeKXHfdgWTqD1YICTim2ST7HKcfUScN4aRMSUXn6CLRBhK82DyhjFViPdJmkjllo_cLcGy_14_ZM3Xz7BmitXZwYoNjfkR5Qn3NR86iZRG1Jc9JLyS0BCmxbPfiXwFFTQReBG46TzOB4dH-P2RYNx3e3kwjN0P5hZWvOZgwzOa0BVYFEqcRi_hjRlFwxXMKMOrhDlfl35GZ_oVTfnam-uNFpR5u4aEriuJqTXDi7I5L91W-l0qb-xWCPHzoh57cfolFMfUL2N2jvhN2Qt9omx4Ojg86qxRQjcoPRkNToff0u06fE3oc4yAxW_W0fHhKD05fv0Lg0HIMQ?type=png)](https://mermaid.live/edit#pako:eNqNVN9P2zAQ_lcsvxS0tKRAB_MDT12lESEq4GWjKHLsa-uR2pntAgXxv-_sJG1gE8IPtnO__N3dl3uhwkigjDr4swYtYKz4wvLVTBNSceuVUBXXnkyBcIe7FVB5ZfQ_-qguudZKL94rs6DMtHksQS4gaKf9s7MpIxfmAciVKYwnSntD_BKfsaDR_F55sQRNHsMmjbFEOWIq0K17xogpfoPwuYUyr4yDvd5YhedveFFC_vP8x-R8Mhn3EtKrAm71AHj_chnxu4T0p-hz7fmqArlP8PUFeNJakhCQmHmEJHihNASMznOsUYCQ9esUOkEYue0prvIG-oEPOFhnzwvuIBegPViEcpsOUoSRDoYJSQej4V0r2m3DQXp3975gCFXGREmM-hbz1hhY3a6ggAewG78MLkbHlLoBogu0-TQ-sqmuI1zL4KJsjO869RcWuIe8NtzrX8YTS3yzqQCPTml2dd8nc-wlcLHc4m6f6kTW8OTfxd22KOhIAc43bkFeKXHfdgWTqD1YICTim2ST7HKcfUScN4aRMSUXn6CLRBhK82DyhjFViPdJmkjllo_cLcGy_14_ZM3Xz7BmitXZwYoNjfkR5Qn3NR86iZRG1Jc9JLyS0BCmxbPfiXwFFTQReBG46TzOB4dH-P2RYNx3e3kwjN0P5hZWvOZgwzOa0BVYFEqcRi_hjRlFwxXMKMOrhDlfl35GZ_oVTfnam-uNFpR5u4aEriuJqTXDi7I5L91W-l0qb-xWCPHzoh57cfolFMfUL2N2jvhN2Qt9omx4Ojg86qxRQjcoPRkNToff0u06fE3oc4yAxW_W0fHhKD05fv0Lg0HIMQ)

## Data types

### Type/Class
An iri, for example `'http://www.ease-crc.org/ont/SOMA.owl#Table'` or the shortform `soma:'Table'` that denotes a class of objects.

### Object
An iri, for example `'http://www.ease-crc.org/ont/SOMA.owl#Table_MBOLQEWJ'` that denotes an individual object.

### PoseStamped
A list of Frame, Position and Rotation.
Position and Rotation are each a list of length 3 and 4.

`[Frame, [X, Y, Z], [RX, RY, RZ, RW]]`

## Objects

This section describes the predicates that are used to create and get information about objects.
An object is any physical object that has a proper space region. Available objects types are defined in the [SUTURO Objects](/objects/).

### Create an object

Create an object of a given Type at the given PoseStamped.

The `Type` can be the full IRI or the namespace:'Name' form.

The Options that can be processed are:
- `shape(ShapeTerm)` - optional, the shape the object has. If not specified, knowledge will have no shape information about this object.
- class_confidence -  a confidence that a robot has about the recognition of objects
```prolog
create_object(-Object, +Type, +PoseStamped) is det.
create_object(-Object, +Type, +PoseStamped, +Options) is det.
```

Example:
```prolog
?- create_object(Object, 'http://www.ease-crc.org/ont/SOMA.owl#CerealBox', ['iai_kitchen/long_table:table:table_front_edge_center', [0,1,1], [0,0,0.70711,0.70711]]).
Object: http://www.ease-crc.org/ont/SOMA.owl#CerealBox_LTKIUPNG.
?- create_object(Object, soma:'CerealBox', ['iai_kitchen/long_table:table:table_front_edge_center', [0,1,1], [0,0,0.70711,0.70711]], [shape(box(0.1,0.1,0.1))]).
Object: http://www.ease-crc.org/ont/SOMA.owl#CerealBox_BHVKCONR.
```

### Pose and shape information

```prolog
object_pose(+Object, -PoseStamped) is semidet.
```

Example:
```prolog
?- object_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_LTKIUPNG', Pose)
Pose: ['iai_kitchen/tall_table:table:table_front_edge_center', [0,0,0], [0,0,0,1]]
```

Because of [knowrob#368](https://github.com/knowrob/knowrob/issues/368) this is currently not done via `object_shape/5` but via `object_shape_workaround/5`.

The documentation of `object_shape/5` is viewable [here](https://knowrob.github.io/knowrob/master/model/SOMA.html#object_shape/5).

```prolog
object_shape_workaround(?Obj, ?Frame, ?ShapeTerm, ?Pose, ?Material) is semidet.
```

Example:
```prolog
?- object_shape_workaround(soma:'CerealBox', Frame, ShapeTerm, Pose, Material).
Frame: CerealBox_BHVKCONR,
Material: {'term': ['material', {'term': ['rgba', '_', '_', '_', '_']}]},
Pose: ['CerealBox_BHVKCONR', '_', '_'],
ShapeTerm: {'term': ['box', 0.1, 0.1, 0.1]}.
```
Note that this is not the format you will see in lisp, there the `term` stuff is probably translated to a lispier structure, like `(box 0.1 0.1 0.1)`.
There might be a better predicate for this in the future, and the blanks in the pose might be a real position and rotation in the future. The `_` indicates that there is no value for this place.

### Poses for robot interaction

Gets a position relative to the object based on the type of relation.

Valid `Type`s are 
 - `perceive`: (Optimal) Pose for the robot to position at to perceive the object
 - `interact`: (Optimal) Pose for the robot to position at to interact with the object (eg. grasp with the gripper)
 - `destination`: Returns the destination pose of an object. The destination pose is the pose where the object should be placed based on their predefined locations and current context. The destination might change over time.
 
Valid entries in the Option list are
- `direction(Dir)` with `Dir` being one of `['-x', '+x', '-y', '+y']`. This Option specifies the direction from which the robot should approach the object.
  For example, if `direction('-x')` is set, the position will have an x value that is smaller than the x value of the object.
  Currently this is only implemented for perceive.
  The default is `direction('-x')`.
 
 This list might be expanded later.

```prolog
object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
object_rel_pose(+Object, +Type, +Options, -PoseStamped) is semidet.
```


Example:
```prolog
?- object_rel_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_YTORLZXJ', perceive, [direction('-x')], Pose).
Pose: ['iai_kitchen/long_table:table:table_front_edge_center', [-0.7, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]].

?- object_rel_pose('http://www.ease-crc.org/ont/SOMA.owl#CerealBox_JDHUPSME', destination, Pose).
Pose: ['iai_kitchen/shelf:shelf:shelf_base_center', [0.0, -0.1, 0.51], [0.0, 0.0, 0.0, 1.0]].
```

For more details, see [`src/model/object/object_rel_pose/README.md`](src/model/object/object_rel_pose/README.md).

### Predefined object locations

Get the predefined origin and destination location of object classes.
The predefined locations are the location (or reference object) where the object is placed at the beginning of the task or should be placed at the end of the task.

```prolog
predefined_origin_location(+Object, -Location) is semidet.
predefined_destination_location(+Class, -DestinationLocation) is semidet.
```

!!! warning
    For `Serving Breakfast` the predicate `init_object_info_serving_breakfast.` has to be called first to load/initialize the challenge specific predefined locations.

Example:
```prolog
?- predefined_origin_location(soma:'Bowl', OriginLocation).
OriginLocation: 'http://www.ease-crc.org/ont/SOMA.owl#Table_FNSVGYRI'.

?- predefined_origin_location(soma:'Bowl', DestinationLocation).
DestinationLocation: 'http://www.ease-crc.org/ont/SOMA.owl#Table_BPXIQGES'.
```

### Semantic similarity measure

The semantic similarity measure is useful for sorting and grouping objects by similarity or category.

#### Most similar object

Finds the most similar object to the given object from a list of input objects.

```prolog	
most_similar_object(+Object, +InputObjects, -MostSimilarObject) is semidet.
```

Example:
```prolog
?- most_similar_object('http://www.ease-crc.org/ont/SUTURO.owl#Strawberry_FDMTIOJK', ['http://www.ease-crc.org/ont/SOMA.owl#CerealBox_QHUCMGZP', 'http://www.ease-crc.org/ont/SUTURO.owl#Banana_WRQHESGO', 'http://www.ease-crc.org/ont/SOMA.owl#Knife_SZIFXLCO'], Object).

Object: 'http://www.ease-crc.org/ont/SUTURO.owl#Banana_WRQHESGO'.
```

#### Wu-Palmer similarity

Calculates the Wu-Palmer similarity between two classes. The similarity can be 0 < similarity <= 1.  

```prolog
wu_palmer_similarity(Class1, Class2, Similarity) is semidet.
```

Example:
```prolog
?- wu_palmer_similarity(suturo:'Banana', suturo:'Strawberry', Similarity).
Similarity: 0.875.
```

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

### Getting the TF frame

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