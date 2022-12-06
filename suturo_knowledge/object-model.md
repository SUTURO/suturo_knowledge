# Object Model

## Namespaces

- `soma_home` is `http://www.ease-crc.org/ont/SOMA-HOME.owl#`

## Attributes that are used in both Furniture 

- Objects are instances of a class (for example `has_type(Object, soma_home:'CerealBox')`)
- the last seen or known location is stored in a way so it can be restored via `tf_get_pose`, for example using `tf_set_pose`.

## Attributes that are used only for percieved objects
- The confidence that perception has is not stored for now.

## Attributes that are used only for furniture from the semantic map
- TODO
