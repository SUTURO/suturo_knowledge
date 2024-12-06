:- module(room_relations,
          [ is_entry_to(r,r),
            is_exit_from(r,r),
            room_entry(r,-),
            room_exit(r,-),
            is_inside_of(r,r),
            room_start_position(r,-,-),
            average(+,-),
            abstand(+,+,-),
            abweichung(+,+,-),
            rsp(r,-),
            optimale_position(+,+,-),
            are_neighbours(-,-,-),
            shortest_path_d(r,r,-,-,-),
            are_neighbours2(-)
          ]).

:- use_module(library(clpfd)).

is_entry_to(Location, Room) ?+>
    triple(Location, suturo:isEntryTo, Room).

is_exit_from(Location, Room) ?+>
    triple(Location, suturo:isExitFrom, Room).

room_entry(Room, Pose) ?>
    is_entry_to(Entry, Room),
    is_at(Entry, Pose).

room_exit(Room, Pose) ?>
    is_exit_from(Exit, Room),
    is_at(Exit, Pose).

is_inside_of(Object, Room) ?+>
    triple(Object, soma:isInsideOf, Room).

% get a starting pose in a room without collision
room_start_position(Room, AvgX,AvgY) :-
    findall([X,Y], 
    (   
        is_inside_of(Object, Room),
        object_pose(Object, [map, [X,Y,_], _])
    ), 
    XYs),
    transpose(XYs, [Xs, Ys]),
    average(Xs, AvgX),
    average(Ys, AvgY).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% baue liste aus Xs und Ys, dann erst average 
% if in einem Objekt, weil 3 Tisch ein einer Reihe

average(List, Average):- 
    sumlist(List, Sum),
    length(List, Length),
    Length > 0, 
    Average is Sum / Length.


% Eine andere Möglichkeit ist die Suche nach einem Punkt,
%  bei dem die maximale Abweichung der Abstände zu allen 
%   anderen Punkten minimiert wird. 

rsp(Room, D) :-
    findall((X,Y), 
    (   
        is_inside_of(Object, Room),
        object_pose(Object, [map, [X,Y,_], _])
    ), 
    XYs),
    abstand(X,Y,D).

abstand((X1, Y1), (X2, Y2), D) :-
    writeln(X1),
    writeln(Y1),
    A is (X2 - X1) * (X2 - X1),
    B is (Y2 - Y1) * (Y2 - Y1),
    I is A + B,
    D is sqrt(I).

abweichung(P, Punkte, Abweichung) :-
    findall(D, (member(Pi, Punkte), 
    abstand(P, Pi, D)), Distanzen), 
    min_list(Distanzen, MinD),
    max_list(Distanzen, MaxD),
    Abweichung is MaxD - MinD.

optimale_position(P, Punkte, BesteAbweichung) :-
    zwischen(-100, 100, X),  % Grenzen definieren
    zwischen(-100, 100, Y),
    P = (X, Y),
    abweichung(P, Punkte, BesteAbweichung).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% determine neighbouring rooms using entry and exit points
% setof könnte verwendet werden, um distinct zu machen, je nachdem, ob gerichtete Kanten oder not
% Cost ggf noch anpassen, aktuell: Anzahl an Exits die nacheinander angesteuert werden müssen

% are_neighbours(-,-,-)
are_neighbours(Room, Boom, E) :-
    is_room_2(Room),
    is_room_2(Boom),
    is_exit_from(E, Room),
    check_inside_room(E, Boom).

% are_neighbours(-,)
are_neighbours2(Results) :-
    findall([Room, Boom, E], (
        is_room_2(Room), 
        is_room_2(Boom), 
        is_exit_from(E, Room), 
        check_inside_room(E, Boom)
        ), Results).

% shortest path with dijkstra
% shortest_path_d(r,r,-,-,-)
% example: is_kitchen(K), is_dining_room(D), shortest_path_d(K,D,P,E,C)
shortest_path_d(Start, Goal, Path, Exits, Cost) :-
    dijkstra([node(Start, [], [], 0)], [], Goal, Path, Exits, Cost).

% dijkstra algorithm for determining shortest path
dijkstra([node(Goal, Path, Exits, Cost)|_], _, Goal, FinalPath, FinalExits, Cost) :-
    reverse([Goal|Path], FinalPath), % Abbruchbedingung
    reverse(Exits, FinalExits). 

dijkstra([node(Current, Path, Exits, Cost)|Queue], Visited, Goal, FinalPath, FinalExits, FinalCost) :-
    findall(node(Next, [Current|Path], [Exit|Exits], NewCost),
        ( are_neighbours(Current, Next, Exit), % neighbours 
            \+ memberchk(node(Next, _, _,_), Visited), % not visited nodes
            NewCost is Cost + 1 % cost+1
        ),
        Neighbors),
    append(Queue, Neighbors, NewQ),
    sort(4, @=<, NewQ, SortedQ), % sort by cost
    dijkstra(SortedQ, [node(Current, Path, Exit, Cost)|Visited], Goal, FinalPath, FinalExits, FinalCost).
