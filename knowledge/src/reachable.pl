:- module(reachable,
    [
    all_objects_graspable/1
    all_objects_not_graspable/1,
    next_graspable_object_on_surface/2,
    all_not_graspable_objects_on_surface/2,
    set_not_graspable/2
    ]).

%%% =============================== reachable objects for knowledge client
%% all_objects_graspable(Graspable) is nondet.
%
%
%
%
all_objects_graspable(Graspable):-
    findall(Subject,ask(triple(Subject, hsr_objects:'hasReachability',0)),Graspable).
%% all_objects_not_graspable(Graspable) is nondet.
%
%
%
%
all_objects_not_graspable(NotGraspable):-
    findall(Subject,ask(triple(Subject, hsr_objects:'hasReachability',>0)),NotGraspable).
%% next_graspable_object_on_surface(NextGraspable, Surface) is nondet.
%3
%
%
%
next_graspable_object_on_surface(NextGraspable, Surface):-
    findall(Subject,ask(triple(Subject, hsr_objects:'supportedBy', Surface)),ObjectsOnSurface),
    predsort(compareDistances, ObjectsOnSurface, SortedObjs),
    nth0(0, SortedObjs, NextGraspable).
%% all_not_graspable_objects_on_surfacte(Graspable, Surface) is nondet.
%
%
%
%
all_not_graspable_objects_on_surfacte(NotGraspable, Surface):-
    findall(Subject,ask(triple(Subject, hsr_objects:'supportedBy', Surface)),ObjectsOnSurface),

%% set_not_graspable(Object, ReachabilityEnum) is det.
%
%
%
%
set_not_graspable(Object, ReachabilityEnum):-
    tell(triple(Object, hsr_objects:'hasReachability', ReachabilityEnum)).

%%% =========================== helper functions
%% calc_distance_from_surface(ObjID, Surface, Distance) is det.
%
%
%
%
calc_distance_from_surface(ObjID, Surface, Distance) :-
    % todo : refactor with existing functionalities
    is_at(Surface, [map,[AX,AY,AZ],_]),
    is_at(ObjID, [map,[BX,BY,BZ],_]),
    DX is AX - BX,
    DY is AY - BY,
    DZ is AZ - BZ,
    Distance is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)).
%% check_if_graspable(Object) is nondet.
%
%
%
%
check_if_graspable(Object) :-
    ask(triple(Object,hsr_objects:'hasReachability', 0)).