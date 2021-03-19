:- module(reachable,
    [
    all_objects_graspable/0,
    all_objects_not_graspable/0,
    next_graspable_object_on_surface/0,
    all_not_graspable_objects_on_surface/0,
    set_not_graspable/1
    ]).

all_objects_graspable(Graspable):-
    ask(triple(Graspable, hsr_objects:'hasReachability', 1)).

all_objects_not_graspable(Graspable):-
    ask(triple(Graspable, hsr_objects:'hasReachability', >(1))).

next_graspable_object_on_surface(Graspable, Surface):-
    !.

all_not_graspable_objects_on_surfacte(Graspable, Surface):-
    !.

set_not_graspable(Object, Reachability):-
    tell(triple(Object, hsr_objects:'hasReachability', Reachability)).