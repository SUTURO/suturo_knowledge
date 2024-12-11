:- module(room_relations,
          [ is_entry_to(r,r),
            is_exit_from(r,r),
            room_entry(r,-),
            room_exit(r,-),
            is_inside_of(r,r),
            rsp(r,-),
            are_neighbours(-,-,-,-),
            shortest_path_d(r,r,-,-,-),
            are_neighbours2(-),
            format_pose(+,-),
            distance_between_rooms(+,+,-)
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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% find a starting point that does not collide with any objects in the room
rsp(Room, Pose) :-
    findall((X,Y), 
    (   
        is_inside_of(Object, Room),
        object_pose(Object, [map, [X,Y,_], _])
    ), 
    XYs),
    best_place(XYs,D),
    format_pose(D,Pose).

% create grid points based on object coordinates
% delete occupied points
best_place(Objects, BestPoint) :-
    generate_grid_from_objects(Objects, 0.5, Grid), 
    exclude(occupied_point(Objects), Grid, FreePoints),
    find_best_point(FreePoints, Objects, BestPoint).

% determine min and max expansion based on object coordinates
generate_grid_from_objects(Objects, Step, Grid) :-
    findall(X, member((X, _), Objects), Xs),
    findall(Y, member((_, Y), Objects), Ys),
    min_list(Xs, MinX), max_list(Xs, MaxX),
    min_list(Ys, MinY), max_list(Ys, MaxY),
    % grid with boundaries 
    findall((X, Y),
            (   my_between(0, ceil((MaxX - MinX) / Step), IX),
                my_between(0, ceil((MaxY - MinY) / Step), IY),
                X is MinX + IX * Step,
                Y is MinY + IY * Step
            ),
            Grid).

% check, whether there is something on this point
% and keep a distance of 50 cm to objects
occupied_point(Objects, (X, Y)) :-
    member((ObjX, ObjY), Objects),
    distance((ObjX, ObjY), (X, Y), Dist),
    Dist < 0.5. 

% find the best point to declare as starting point in a room
find_best_point([Point], Objects, Point) :- !.
find_best_point([Point | Rest], Objects, BestPoint) :-
    max_distance_to_objects(Point, Objects, MaxDist),
    find_best_point(Rest, Objects, OtherBestPoint),
    max_distance_to_objects(OtherBestPoint, Objects, OtherMaxDist),
    (   MaxDist < OtherMaxDist -> 
        BestPoint = Point ; 
        BestPoint = OtherBestPoint 
    ).

% find the max distance to all objects in the room
max_distance_to_objects(Point, Objects, MaxDist) :-
    findall(Dist, 
                (member(Obj, Objects),
                distance(Point, Obj, Dist)
                ), Distances),
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


% format a pose where you only get x and y coordinates
format_pose((X, Y), ['map', [X, Y, 0], [0, 0, 0, 1]]).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% determine neighbouring rooms using entry and exit points
% setof könnte verwendet werden, um distinct zu machen, je nachdem, ob gerichtete Kanten oder not
% Cost ggf noch anpassen, aktuell: Anzahl an Exits die nacheinander angesteuert werden müssen

% are_neighbours(-,-,-,-)
are_neighbours(Room, Boom, E, Distance) :-
    is_room_2(Room),
    is_room_2(Boom),
    is_exit_from(E, Room),
    check_inside_room(E, Boom),
    distance_between_rooms(Room, Boom, Distance).

% are_neighbours2(-)
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
    dijkstra([node(Start, [], [], 0)], [], Goal, Path, Exits, Cost), !.

    dijkstra([], _, _, _, _, _) :- 
        writeln('No path found'), fail.
    
    dijkstra([node(Goal, Path, Exits, Cost)|_], _, Goal, FinalPath, FinalExits, Cost) :-
        reverse([Goal|Path], FinalPath), 
        reverse(Exits, FinalExits).
    
    dijkstra([node(Current, Path, Exits, Cost)|Queue], Visited, Goal, FinalPath, FinalExits, FinalCost) :-
        findall(node(Next, [Current|Path], [Exit|Exits], NewCost),
            ( are_neighbours(Current, Next, Exit, Distance),
              \+ memberchk(Next, Path),
              %\+ memberchk(node(Next, _, _, _), Visited),
              NewCost is Cost + Distance
            ),
            Neighbors),
            writeln(['Neighbors:', Neighbors]),
        ( Neighbors = [] ->
            writeln(['No neighbors for:', Current]),
            dijkstra(Queue, Visited, Goal, FinalPath, FinalExits, FinalCost)
        ; true),
        append(Queue, Neighbors, NewQueue),
        %writeln(['NewQueue:', NewQueue]),
        sort(4, @=<, NewQueue, SortedQueue),
        dijkstra(SortedQueue, [node(Current, Path, Exits, Cost)|Visited], Goal, FinalPath, FinalExits, FinalCost).
    
% determine euclidean distance between starting points of 2 rooms 
distance_between_rooms(Room1, Room2, Distance) :-
    rsp(Room1, ['map', [X1, Y1, _], _]),
    rsp(Room2, ['map', [X2, Y2, _], _]),
    distance((X1, Y1), (X2, Y2), Distance).
        