:- module(path_finder,
    [
        shortest_path_between_rooms/3
    ]).


:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).

:- use_module(library('model/environment/doors'),[get_door_state/2]).
:- use_module(library('locations/spatial_comp'), [euclidean_distance/3]).

shortest_path_between_rooms(OriginRoom, DestinationRoom, Path) :-
    robot_velocity(Velocity),
    door_opening_time(OpeningTime),
    findall([OriginLinkage, DestinationLinkage],
    (
        triple(OriginLocation, soma:'isLinkedTo', OriginRoom),
        triple(DestinationLocation, soma:'isLinkedTo', DestinationRoom),
        triple(OriginLinkage, dul:'hasLocation', OriginLocation),
        triple(DestinationLinkage, dul:'hasLocation', DestinationLocation)
    ),
    OriginDestinationPairs),
    findall([Costs, PossiblePath], 
    (
        member([Origin, Destination], OriginDestinationPairs),
        get_urdf_origin(Map),
        has_urdf_name(Origin, OriginLink),
        tf_lookup_transform(Map, OriginLink, pose(OriginPosition, _)),
        tf_lookup_transform(Map, 'base_footprint', pose(RobotPosition, _)),
        euclidean_distance(OriginPosition, RobotPosition, InitialDistance),
        ((has_type(Origin, hsr_rooms:'Door'), get_door_state(Origin, 0))
        -> InitialCosts is InitialDistance/Velocity + OpeningTime
        ; InitialCosts is InitialDistance/Velocity),
        a_star(Origin, InitialDistance, Destination, PossiblePath, Costs)
    ), 
    PossiblePaths),
    min_member([_, Path], PossiblePaths).



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%% A* Algorithm %%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

a_star(Origin, DistanceToOrigin, Destination, Path, Costs) :-
    OpenList = [Origin],
    ClosedList = [],
    tell(has_type(Attributes, hsr_rooms:'AStarAttributes')),
    tell(triple(Origin, hsr_rooms:'hasAStarAttr', Attributes)),
    tell(triple(Attributes, hsr_rooms:'hasGValue', DistanceToOrigin)),
    next_node(Destination, OpenList, ClosedList),
    InitialPath = [Destination],
    path(Origin, Destination, InitialPath, Path, Costs),
    cleanup_a_star_attributes.


next_node(_, [], _).
next_node(Destination, OpenList, ClosedList) :-
    min_distance(OpenList, CurrentNode),
    delete(OpenList, CurrentNode, TempOpenList),

    (same_as(CurrentNode, Destination)
    -> next_node(Destination, [], ClosedList)
    ;(
        append([CurrentNode], ClosedList, NewClosedList),
        expand_node(CurrentNode, Destination, TempOpenList, NewClosedList, NewOpenList),
        next_node(Destination, NewOpenList, NewClosedList)
    )).


expand_node(CurrentNode, Destination, OpenList, ClosedList, NewOpenList) :-
    findall(SuccessorNode,
    (
        triple(Path, hsr_rooms:'hasOrigin', CurrentNode),
        triple(Path, hsr_rooms:'hasDestination', SuccessorNode),
        not member(SuccessorNode, ClosedList)
    ),
    Successors),
    g_value(CurrentNode, CurrentG),
    forall(member(SuccessorNode, Successors),
    (
        path_costs(CurrentNode, SuccessorNode, PathCosts),
        NewG is CurrentG + PathCosts,
        ((not member(SuccessorNode, OpenList); 
        (g_value(SuccessorNode, OldG), NewG < OldG))
        -> 
        (
            heuristic(SuccessorNode, Destination, Distance),
            F is NewG + Distance,
            update_a_star_attributes(SuccessorNode, CurrentNode, NewG, F)
        );
            ros_info("A* No cheaper path found")
        )
    )),
    union(Successors, OpenList, NewOpenList), 
    !.


min_distance(OpenList, NextNode) :-
    findall([GValue, Node], (member(Node, OpenList), g_value(Node, GValue)), NodeDistances),
    min_member([_, NextNode], NodeDistances).


path_costs(Origin, Destination, PathCosts) :-
    triple(Path, hsr_rooms:'hasOrigin', Origin),
    triple(Path, hsr_rooms:'hasDestination', Destination),    
    triple(Path, hsr_rooms:'hasCosts', PathCosts).

g_value(Node, G) :-
    triple(Node, hsr_rooms:'hasAStarAttr', Attributes),
    triple(Attributes, hsr_rooms:'hasGValue', G).

heuristic(Origin, Destination, Distance) :-
    get_urdf_origin(Map),
    has_urdf_name(Origin, OriginLink),
    tf_lookup_transform(Map, OriginLink, pose(OriginPosition, _)),
    has_urdf_name(Destination, DestinationLink),
    tf_lookup_transform(Map, DestinationLink, pose(DestinationPosition, _)),
    euclidean_distance(OriginPosition, DestinationPosition, Distance).


update_a_star_attributes(Node, Predecessor, G, F) :-
    (not triple(Node, hsr_rooms:'hasAStarAttr', _)
    -> (
        tell(has_type(Attributes, hsr_rooms:'AStarAttributes')),
        tell(triple(Node, hsr_rooms:'hasAStarAttr', Attributes))
    );
        triple(Node, hsr_rooms:'hasAStarAttr', Attributes)
    ),
    update(triple(Attributes, hsr_rooms:'hasGValue', G)),
    update(triple(Attributes, hsr_rooms:'hasFValue', F)),
    triple(Predecessor, hsr_rooms:'hasAStarAttr', PredecessorAttributes),
    update(triple(Attributes, hsr_rooms:'hasPredecessor', PredecessorAttributes)).


cleanup_a_star_attributes :-
    forall(triple(Attributes, hsr_rooms:'hasGValue', _), tripledb_forget(Attributes, hsr_rooms:'hasGValue', _)),
    forall(triple(Attributes, hsr_rooms:'hasFValue', _), tripledb_forget(Attributes, hsr_rooms:'hasFValue', _)),
    forall(triple(Attributes, hsr_rooms:'hasGPredecessor', _), tripledb_forget(Attributes, hsr_rooms:'hasPredecessor', _)),
    forall(triple(Node, hsr_rooms:'hasAStarAttr', _), tripledb_forget(Node, hsr_rooms:'hasAStarAttr', _)).


path(Origin, Origin, CurrentPath, Path, Costs) :-
    Path = CurrentPath,
    Costs = 0.

path(Origin, Destination, CurrentPath, Path, Costs) :-
    triple(Destination, hsr_rooms:'hasAStarAttr', DestinationAttributes),
    triple(DestinationAttributes, hsr_rooms:'hasPredecessor', PredecessorAttributes),
    triple(Predecessor, hsr_rooms:'hasAStarAttr', PredecessorAttributes),
    path(Origin, Predecessor, CurrentPath, ExtendedPath, _),
    append([Predecessor], ExtendedPath, Path),
    triple(DestinationAttributes, hsr_rooms:'hasFValue', Costs).
