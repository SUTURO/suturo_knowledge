:- module(object_manipulation, [
    set_object_handeled/1,
    set_object_not_handeled/1,
    update_handle_state/2,
    handeled/1,
    objects_not_handeled/1
]).

:- use_module(library('model/objects/object_info'), [is_suturo_object/1]).

set_object_handeled(Object) :-
    update_handle_state(Object, true).

set_object_not_handeled(Object) :-
    update_handle_state(Object, false).

update_handle_state(Object, State) :-
    is_suturo_object(Object),
    triple(Object, hsr_objects:'hasHandleState', HandleState),
    forall(triple(HandleState, hsr_objects:'handeled', _), tripledb_forget(HandleState, hsr_objects:'handeled', _)),
    tell(triple(HandleState, hsr_objects:'handeled', State)).

handeled(Object) :-
    is_suturo_object(Object),
    triple(Object, hsr_objects:'hasHandleState', HandleState),
    triple(HandleState, hsr_objects:'handeled', State),
    State == 1.

objects_not_handeled(Objects) :-
    findall(Object,
    (
        is_suturo_object(Object),
        not handeled(Object)
    ),
    Objects).
