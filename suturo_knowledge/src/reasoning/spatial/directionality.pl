:- module(directionality,
          [
              sort_right_to_left(+,t,-)
          ]).

:- use_module(library(pairs)).

%% sort_right_to_left(+RefereceFrame, +Objects, -SortedObjects) is det.
%
% sort a list of object from right to left
% (according to the y axis from most negative to most positive)
%
sort_right_to_left(Reference, Objects, Sorted) :-
    \+ atom(Reference),
    !,
    ros_error('no reference frame in sort_right_to_left'),
    fail.
sort_right_to_left(Reference, Objects, Sorted) :-
    \+ ground(Objects),
    !,
    ros_error('non-ground object list in sort_right_to_left'),
    fail.
sort_right_to_left(Reference, Objects, Sorted) :-
    findall(Y-Object,
            (
                member(Object, Objects),
                kb_call(is_at(Object, [Reference, [_,Y,_], _]))
            ),
            Positions),
    sort(1,@=<,Positions,SortedPositions),
    pairs_values(SortedPositions, Sorted).
