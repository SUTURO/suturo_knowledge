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
            are_neighbours3(-,-,-),
            format_pose(+,-),
            distance_between_rooms(+,+,-),
            distance_between_rooms_1(+,+,+,-),
            distance2(+,+,-),
            door_penalty(+,+,+,-),
            astar(+,+,-,-),
            heuristic(+,+,+,-),
            neighbors_from(+,-),
            entry_pose(-, r),
            exit_pose(-, r),
            nav_pose_in_room(+, r,-),
            perceive_pose_in_room(+,r,-)
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

is_room_middle(Room, Pose) ?>
    is_at(Room, Pose).

entry_pose(Room, [map, X,Y]):-
    triple(Location, suturo:isEntryTo, Room),
    object_pose(Location, [map, X,Y]).

exit_pose(Room, [map,X,Y]):-
    triple(Location, suturo:isExitFrom, Room),
    object_pose(Location, [map, X,Y]).

% Object = 'table'
nav_pose_in_room(Object, Room, Poses) :-
    what_object(Object, Obj),
    findall(F, has_type(F, Obj), Objects),  % Alle Instanzen des Objekttyps sammeln
    has_type(RoomInst, Room),               % Room muss eine Instanz eines Raums sein
    findall(Pose,
        (
            member(F, Objects),             % Iteration über alle Instanzen
            is_inside_of(F, RoomInst),      % Prüfen, ob das Objekt im Raum ist
            furniture_rel_pose(F, 'perceive', Pose)  % Pose des Objekts abrufen
        ),
        Poses
    ).

perceive_pose_in_room(Object, Room, [Frame, [NewX,Y,Z], Rotation]) :-
    what_object(Object, Obj),
    has_type(RoomInst, Room),
    has_type(ObjectInst, Obj),
    is_inside_of(ObjectInst, RoomInst),
    object_pose(ObjectInst, [Frame, [X,Y,Z], Rotation]),
    longest_side(ObjectInst, LSize),
    NewX is X - ((LSize/2) + 0.8).

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
    distance2((ObjX, ObjY), (X, Y), Dist),
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
                distance2(Point, Obj, Dist)
                ), Distances),
    max_list(Distances, MaxDist).

% calculate euclidean distance 
distance2((X1, Y1), (X2, Y2), Dist) :-
    DX is X1 - X2,
    DY is Y1 - Y2,
    Dist is sqrt(DX * DX + DY * DY).

% calculate euclidean distance with 3 coordinates
distance3((X1, Y1), (X2, Y2), (X3, Y3), Dist) :-
    DX1 is X1 - X2,
    DX2 is X2 - X3,
    DY1 is Y1 - Y2,
    DY2 is Y2 - Y3,
    Dist is sqrt(DX1 * DX1 + DY1 * DY1 + DX2 * DX2 + DY2 * DY2).

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
    %distance_between_rooms(Room, Boom, Distance).
    distance_between_rooms_1(Room, Boom, E, Distance).

% are_neighbours(-,-,-,-)
are_neighbours3(Room, Boom, E) :-
    is_room_2(Room),
    is_room_2(Boom),
    is_exit_from(E, Room),
    check_inside_room(E, Boom).

% are_neighbours2(-)
are_neighbours2(Results) :-
    findall([Room, Boom, E], (
        is_room_2(Room), 
        is_room_2(Boom), 
        is_exit_from(E, Room), 
        check_inside_room(E, Boom)
        ), Results).

neighbors_from(StartRoom, List) :-
    findall([Exit, Neighbors],  
        (is_exit_from(Exit, StartRoom),
        writeln(['is_exit_from:', Exit]),
        object_pose(Exit, ['map', [X1, Y1, _], _]),
        is_entry_to(Entry, Neighbors),
        writeln(['is_entry_from:', Entry]),
        object_pose(Entry, ['map', [X2, Y2, _], _]),
        X1 =:= X2,
        writeln(['Werte X :', X1 =:= X2]),
        Y1 =:= Y2,
        writeln(['Werte Y :', Y1 =:= Y2])),
    List),
    writeln(['Generierte Liste:' , List]).


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
              \+ memberchk(node(Next, _, _, _), Visited),
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
% distance_between_rooms(+,+,-)
distance_between_rooms(Room1, Room2, Distance) :-
    rsp(Room1, ['map', [X1, Y1, _], _]),
    rsp(Room2, ['map', [X2, Y2, _], _]),
    distance2((X1, Y1), (X2, Y2), Distance).
        
% determine distance from door to door as sum of distance from door1 to room middle and room middle to door2
% distance_between_rooms_1(+,+,+,-)
distance_between_rooms_1(StartRoom, GoalRoom, Exit, Distance) :-
    object_pose(StartRoom, ['map', [X1, Y1, _], _]),
    object_pose(Exit, ['map', [X2, Y2, _], _]),
    object_pose(GoalRoom, ['map', [X3, Y3, _], _]),
    distance3((X1, Y1), (X2, Y2), (X3, Y3),  Distance).

% gives a penalty for narrow passages between rooms
% door_penalty(+,+,+,-)
door_penalty(StartRoom, GoalRoom, StartExit, Penalty) :-
    (StartRoom = GoalRoom -> 
        Penalty is 0
    ;
    writeln(['StartRoom:', StartRoom]),
    is_exit_from(StartExit, StartRoom),
    is_entry_to(EntryStart, StartRoom),
    is_exit_from(ExitGoal, GoalRoom),
    object_pose(EntryStart, ['map', [X1, Y1, _], _]),
    object_pose(ExitGoal, ['map', [X2, Y2, _], _]),
    X1 =:= X2, 
    Y1 =:= Y2,
    is_entry_to(EntryGoal, GoalRoom),
    object_pose(EntryGoal, ['map', [X3, Y3, _], _]),
    object_pose(StartExit, ['map', [X4, Y4, _], _]),
    X3 =:= X4,
    Y3 =:= Y4,
    distance2((X1,Y1), (X3,Y3), Distance),
    calculate_penalty(Distance, Penalty)).

% calculates a penalty score depending on width of passage 
% calculate_penalty(+,-)
calculate_penalty(Distance, Penalty) :-
    Reference is 0.84,
    ( Distance < Reference ->
        Penalty is Reference / Distance % Für Distance < 0.84: Penalty wächst
    ;
        Penalty is Distance / Reference % Für Distance >= 0.84: Penalty sinkt
    ).
        

% A* Algorithmus
astar(Start, Goal, Path, Cost) :-
    astar_search([node(Start, [], 0)], [], Goal, Path, Cost).

astar_search([node(Current, Path, Cost)|_], _, Current, FinalPath, Cost) :-
    reverse([Current|Path], FinalPath).

astar_search([node(Current, Path, Cost)|Queue], Visited, Goal, FinalPath, FinalCost) :-
    % find neighbors
    findall(node(Next, [Current|Path], NewCost),
        (   are_neighbours3(Current, Next, Exit),
            \+ memberchk(Next, Path),
            \+ memberchk(node(Next, _, _, _), Visited),
            heuristic(Current, Next, Exit, NewCost)
        ),
        Neighbors),

    sort(3, @=<, Neighbors, NewQueue),
    astar_search(NewQueue, [node(Current, Path, Cost)|Visited], Goal, FinalPath, FinalCost).


% heuristic: combination of door_penalty and distance_between_rooms_1
% heuristics(+,+,+,-)
heuristic(CurrentRoom, GoalRoom, Exit, Heuristic) :-
    door_penalty(CurrentRoom, GoalRoom, Exit, Penalty),
    distance_between_rooms_1(CurrentRoom, GoalRoom, Exit, Distance),
    Heuristic is Penalty + Distance.
