# Object Model

## Namespaces

- `soma` is `http://www.ease-crc.org/ont/SOMA.owl#`
- `suturo` is `http://www.ease-crc.org/ont/SUTURO#`

## Attributes that are used in both Furniture and perceived objects

- Objects are instances of a class (for example `has_type(Object, soma:'CerealBox')`)
- the last seen or known location is stored in a way so it can be restored via `tf_get_pose`, for example using `tf_set_pose`.

## Attributes that are used only for percieved objects
- The confidence that perception has is not stored for now.

## Furniture only
- Tables are an instance of `soma:'Table'`
- The shape of a Furniture is stored in a way that `'model_SOMA':object_shape/5` can find the correct information.
