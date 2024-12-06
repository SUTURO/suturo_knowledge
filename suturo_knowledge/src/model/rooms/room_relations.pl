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
% baue Liste aus Xs und Ys, dann erst average 
% if in einem Objekt, weil 3 Tische in einer Reihe

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
    best_place(XYs,D).

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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

best_place(Objects, BestPoint) :-
    generate_grid_from_objects(Objects, 0.5, Grid), % Erstelle Gitterpunkte basierend auf den Objektkoordinaten
    exclude(occupied_point(Objects), Grid, FreePoints), % Entferne belegte Punkte
    find_best_point(FreePoints, Objects, BestPoint).


generate_grid_from_objects(Objects, Step, Grid) :-
    % determine min and max Ausdehnung based on object coordinates
    findall(X, member((X, _), Objects), Xs),
    findall(Y, member((_, Y), Objects), Ys),
    min_list(Xs, MinX), max_list(Xs, MaxX),
    min_list(Ys, MinY), max_list(Ys, MaxY),
    % grid with Grenzen 
    findall((X, Y),
            (   my_between(0, ceil((MaxX - MinX) / Step), IX),
                my_between(0, ceil((MaxY - MinY) / Step), IY),
                X is MinX + IX * Step,
                Y is MinY + IY * Step
            ),
            Grid).

% check, whether there is something on this point
occupied_point(Objects, (X, Y)) :-
    member((ObjX, ObjY), Objects),
    distance((ObjX, ObjY), (X, Y), Dist),
    Dist < 0.5. % keep distance of 50 cm to objects

% find the best point to declare as starting point in a room
find_best_point([Point], Objects, Point) :- !.
find_best_point([Point | Rest], Objects, BestPoint) :-
    max_distance_to_objects(Point, Objects, MaxDist),
    find_best_point(Rest, Objects, OtherBestPoint),
    max_distance_to_objects(OtherBestPoint, Objects, OtherMaxDist),
    (   MaxDist < OtherMaxDist -> BestPoint = Point ; BestPoint = OtherBestPoint ).

% find the max distance to all objects in the room
max_distance_to_objects(Point, Objects, MaxDist) :-
    findall(Dist, (member(Obj, Objects), distance(Point, Obj, Dist)), Distances),
    max_list(Distances, MaxDist).

% calculate euclidean distance 
distance((X1, Y1), (X2, Y2), Dist) :-
    DX is X1 - X2,
    DY is Y1 - Y2,
    Dist is sqrt(DX * DX + DY * DY).

% like "between": generate values within a certain value range
% for trying to find a suitable value 
my_between(Lower, Upper, Lower) :-
    Lower =< Upper.
my_between(Lower, Upper, X) :-
    Lower < Upper,
    Next is Lower + 1,
    my_between(Next, Upper, X).


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
