# Planning API Documentation

## Quick lookup for planning

Dropping the database: execute this in the shell and restart knowledge.
```bash
mongo roslog --eval "db.dropDatabase()"
```

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
[![](https://mermaid.ink/img/pako:eNqNVF1v2jAU_SuWXwA10EBLo_qhD1tXaWVVUfu0lSpynAv1CHZmm34M9b_v2gkQaDURCTvxPffk5J7LXVGhc6CMWvizBCXgUvKZ4YuJIqTkxkkhS64cGQPhFlcjoHRSqw_xEC64UlLN9oMjHxwp_VJAPgMfHXcvLkaMSCVdasE8Y1KaGeDzKbduDRgzcqOfgdzpTDvEOk3cE-owoJBvLp14AkVe_JJrbYi0RJegGvylgRymUkGeaiNnUqWFFtzrb1u94Kz1BTW1ItIdb4A_akCH4Otm4BocZJ1M9JRkmGn9q0bdSupHCka-8gwPXPrz-vvV9dXVZUOazn6DcKmBIi21hXZrH4uyWqWvt3wGvD-6DXW3Xizi7x1flJBvVdZI4sm8Pl8pUVFi6azj6O2O3C0JIw8tyWVaV_S4TmN7e5pxC6kA5cCgoIe4F6OYuNePSNwb9h_XR9ul34sfH_fdRMFrXTuiNzhgVZ_5QKiy1y_zzecEIKw_o0bmAUq4yol0docQiy2wtRykVc3b3duwR2SnB44aFdmWu1PnVvSfFbIiY8TTHGT0DjC4XHBxgMU5WCdV1YDNTvROe3UHuut4VgBrrP_19ewQX8dSzDcyggfhk9AJwl0IfKq9jT8fDCo6Db47KKFO5JnvGetw6ljc_FBRM89al7XxD1_zHg_IFKeBz9a4mBpqSduWWquICEBLi0y_RmQhi3nJxbxDI7oAs-Ayx1m48lomFJMXMKEMb_EdfFm4CZ2od4TypdP3b0pQ5swSIrosc-yRenRSNuWF3Zx-y6XTZnMI4fGmGrph9kYUh-QvrbeJ-EzZir5SdnLWOz1rXElE3yjrD897gzhJkkGM22B4-h7Rv4EAnamuYXyenCT9JHn_B7kt86Q?type=png)](https://mermaid.live/edit#pako:eNqNVF1v2jAU_SuWXwA10EBLo_qhD1tXaWVVUfu0lSpynAv1CHZmm34M9b_v2gkQaDURCTvxPffk5J7LXVGhc6CMWvizBCXgUvKZ4YuJIqTkxkkhS64cGQPhFlcjoHRSqw_xEC64UlLN9oMjHxwp_VJAPgMfHXcvLkaMSCVdasE8Y1KaGeDzKbduDRgzcqOfgdzpTDvEOk3cE-owoJBvLp14AkVe_JJrbYi0RJegGvylgRymUkGeaiNnUqWFFtzrb1u94Kz1BTW1ItIdb4A_akCH4Otm4BocZJ1M9JRkmGn9q0bdSupHCka-8gwPXPrz-vvV9dXVZUOazn6DcKmBIi21hXZrH4uyWqWvt3wGvD-6DXW3Xizi7x1flJBvVdZI4sm8Pl8pUVFi6azj6O2O3C0JIw8tyWVaV_S4TmN7e5pxC6kA5cCgoIe4F6OYuNePSNwb9h_XR9ul34sfH_fdRMFrXTuiNzhgVZ_5QKiy1y_zzecEIKw_o0bmAUq4yol0docQiy2wtRykVc3b3duwR2SnB44aFdmWu1PnVvSfFbIiY8TTHGT0DjC4XHBxgMU5WCdV1YDNTvROe3UHuut4VgBrrP_19ewQX8dSzDcyggfhk9AJwl0IfKq9jT8fDCo6Db47KKFO5JnvGetw6ljc_FBRM89al7XxD1_zHg_IFKeBz9a4mBpqSduWWquICEBLi0y_RmQhi3nJxbxDI7oAs-Ayx1m48lomFJMXMKEMb_EdfFm4CZ2od4TypdP3b0pQ5swSIrosc-yRenRSNuWF3Zx-y6XTZnMI4fGmGrph9kYUh-QvrbeJ-EzZir5SdnLWOz1rXElE3yjrD897gzhJkkGM22B4-h7Rv4EAnamuYXyenCT9JHn_B7kt86Q)

### Storing Groceries
[![](https://mermaid.ink/img/pako:eNq1VN9P2zAQ_lcsv5SKNKRAifADD4MxjYKogIcBRZHjXFuP1M5iFyiI_31nJy2haNXQtDzYvp-5-z77XqjQGVBGDfyagRJwJPm45NOhIqTgpZVCFlxZMgDCDa6lgMJKrT7YvTnnSkk1XjX2nbGv9GMO2RicddA5OBgwcqYfgFzoVFsildWEl6A4eZyAIpnWJZGG6ALUagR6ilmRal5mhKvM-7xpXOQios-ITn-CsEkJeVJoAxutw9oxub46vzi9-XHSCkircJ3JB8Dz5rnv0ASkM8CAS8unBWRtl7Lfqapo6Bm5bUkuk3tpBda9ZSaQj1hjTVJuIBGgLJSY_TYKI8wchd2ARGGve7dQvS3dMLq7W9fzoljiGlo6Aqv4cQZ4gHJuJ8gFAmtkBstg7w51H6fSWKJHNUbGg2knIEuf2TRQFCVwC0nluNE59ztidTUvICBNQFCqAWyTEXIIXEyWFWf1r9bxc8XTHCpyvh1drpBD_kzOClrWpfkcVFq57qvI_4LT5jucNj-Fk4Inu5K37focg_U2koIr0hucvpDivnlnqxBGDgFLzL_op-T65PvxyfHx0drH8sHbE5Jz8e9PRfBUKrBsZV_7YPb-5sEMsHXkhy_hcHz5molEwXqWM4RLKu7qJ7kWfDHV6hwXUEDtyVN3pYzFyWfIRoOHLSzJumG1ClsYhg2UUGq3aUCnUE65zHDYvrgfDSnGTmFIGR4zGPFZbod0qF7Rlc-svpwrQZktZxDQWZHhtapn83vl10xaXVI24rlBJXjxrBrqfrYHFIfwjdbTpQ_KlL3QJ8p29sLdvcYXB3ROWbe3H25HcRxvR7ht93ZfA_rsEyDS1deL9uOduBvHr78B5QoMvA?type=png)](https://mermaid.live/edit#pako:eNq1VN9P2zAQ_lcsv5SKNKRAifADD4MxjYKogIcBRZHjXFuP1M5iFyiI_31nJy2haNXQtDzYvp-5-z77XqjQGVBGDfyagRJwJPm45NOhIqTgpZVCFlxZMgDCDa6lgMJKrT7YvTnnSkk1XjX2nbGv9GMO2RicddA5OBgwcqYfgFzoVFsildWEl6A4eZyAIpnWJZGG6ALUagR6ilmRal5mhKvM-7xpXOQios-ITn-CsEkJeVJoAxutw9oxub46vzi9-XHSCkircJ3JB8Dz5rnv0ASkM8CAS8unBWRtl7Lfqapo6Bm5bUkuk3tpBda9ZSaQj1hjTVJuIBGgLJSY_TYKI8wchd2ARGGve7dQvS3dMLq7W9fzoljiGlo6Aqv4cQZ4gHJuJ8gFAmtkBstg7w51H6fSWKJHNUbGg2knIEuf2TRQFCVwC0nluNE59ztidTUvICBNQFCqAWyTEXIIXEyWFWf1r9bxc8XTHCpyvh1drpBD_kzOClrWpfkcVFq57qvI_4LT5jucNj-Fk4Inu5K37focg_U2koIr0hucvpDivnlnqxBGDgFLzL_op-T65PvxyfHx0drH8sHbE5Jz8e9PRfBUKrBsZV_7YPb-5sEMsHXkhy_hcHz5molEwXqWM4RLKu7qJ7kWfDHV6hwXUEDtyVN3pYzFyWfIRoOHLSzJumG1ClsYhg2UUGq3aUCnUE65zHDYvrgfDSnGTmFIGR4zGPFZbod0qF7Rlc-svpwrQZktZxDQWZHhtapn83vl10xaXVI24rlBJXjxrBrqfrYHFIfwjdbTpQ_KlL3QJ8p29sLdvcYXB3ROWbe3H25HcRxvR7ht93ZfA_rsEyDS1deL9uOduBvHr78B5QoMvA)

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

To get or set the pose of an object, use the following predicate.  
If an unset variable is passed for `PoseStamped`, the predicate will try to find a pose in the knowledge base.
If the `PoseStamped` is fully specified, the predicate will set the pose in the knowledge base.

```prolog
object_pose(+Object, ?PoseStamped) is semidet.
```

Example:
```prolog
% Get the pose of an object
?- object_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_LTKIUPNG', Pose)
Pose: ['map', [1,0,1], [0,0,0,1]]

% Set/Update the pose of an object
?- object_pose('http://www.ease-crc.org/ont/SOMA.owl#Table_LTKIUPNG', ['map', [2,1,0], [0,0,0,1]])
true.
```

The documentation of `object_shape/5` is viewable [here](https://knowrob.github.io/knowrob/master/model/SOMA.html#object_shape/5).
Because of [knowrob#368](https://github.com/knowrob/knowrob/issues/368) this is currently not done via `object_shape/5` but via `object_shape_workaround/5`.

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
 
Valid entries in the `Options` list are

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
predefined_origin_location(+Class, -OriginLocation) is nondet.
predefined_destination_location(+Class, -DestinationLocation) is nondet.
```

!!! warning
    For `Serving Breakfast` the predicate `init_serving_breakfast.` has to be called first to load/initialize the challenge specific predefined locations.

Example:
```prolog
?- predefined_origin_location(soma:'Bowl', OriginLocation).
OriginLocation: 'http://www.ease-crc.org/ont/SOMA.owl#Shelf_FNSVGYRI'.

?- predefined_destination_location(soma:'Bowl', DestinationLocation).
DestinationLocation: 'http://www.ease-crc.org/ont/SOMA.owl#Table_BPXIQGES'.
```

### Next best object

Gets the next best object to pick based on the current context.

This reasoner calculates the next best object to pick based on factors like the distance of the object to the robot and the destination location, the benefit (bonus points) and the confidence of the object detection by Perception.

!!! info
    For `next_object` to work, the objects have to be created first with the `create_object` predicate.

```prolog
next_object(-Object) is nondet.
```

Example:
```prolog
?- next_object(Object).

Object: 'http://www.ease-crc.org/ont/SUTURO.owl#Banana_WRQHESGO'.
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
