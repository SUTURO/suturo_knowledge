%% The WuPalmer module contains predicates that can calculate the similarity between two classes of an ontology.
:- module(wu_palmer,
	  [
        wu_palmer_similarity/3,
        path_up/3,
        path_down/3,
        find_path_down/3,
        shortest_path/3,
        superclasses/2,
        subclasses/2,
        direct_superclasses/2,
        direct_subclasses/2
	  ]).

:- use_module(library('semweb/rdf_db')).

:- use_module(library('util/util'),
	      [
		  split_iri/3,
		  ros_warn/2
	      ]).

:- rdf_meta(wu_palmer_similarity(r,r,-)).
:- rdf_meta(path_up(r,r,-)).
:- rdf_meta(path_down(r,r,?)).
:- rdf_meta(find_path_down(r,r,-)).
:- rdf_meta(shortest_path(r,r,-)).
:- rdf_meta(superclasses(r,-)).
:- rdf_meta(subclasses(r,-)).
:- rdf_meta(direct_superclasses(r,-)).
:- rdf_meta(direct_subclasses(r,-)).

%% wu_palmer_similarity(+ClassA, +ClassB, -Similarity) is det.
%
% Calculates the Wu-Palmer similarity between two classes
%
% @param ClassA One of the two classes
% @param ClassB Second of the two classes
% @param Similarity The similarty measure between 0 (not similar) and 1 (most similar)
%
wu_palmer_similarity(ClassA, ClassB, Similarity) :-
    % Get the least common subsumer (LCS)
    % lcs(CA, CB, LCS),
    % ros_warn('LCS: ~w', [LCS]),
    % % Calculate the depth of the LCS and both classes
    % shortest_distance(LCS, dul:'Entity', DepthLCS),
    Similarity is -1.
    % shortest_depth(CA, Depth1),
    % shortest_depth(CB, Depth2),
    % Calculate the Wu-Palmer similarity measure
    % Similarity is (2 * DepthLCS) / (Depth1 + Depth2).

%% lcs(+ClassA, +ClassB, -LCS) is semidet.
%
% Least common subsumer/superclass (LCS) of two classes.
%
% @param ClassA One of the two classes
% @param ClassB Second of the two classes
% @param LCS The least common subsumer of the two classes
%
lcs(ClassA, ClassB, LCS) :-
    % Get the superclasses of both classes
    superclasses(ClassA, SuperClassesA),
    superclasses(ClassB, SuperClassesB),
    % Find the least common subsumer (LCS)
    find_lcs(SuperClassesA, SuperClassesB, LCS).

%% find_lcs(+ClassList1, +ClassList2, -LCS) is semidet.
%
% Helper predicate to find the least common subsumer (LCS).
% ClassesA are expected be ordered from most specific to least specific.
%
find_lcs([Class|_], Classes, Class) :-
    member(Class, Classes),
    !.
find_lcs([_|ClassesA], ClassesB, LCS) :-
    find_lcs(ClassesA, ClassesB, LCS).

%% path_up(+ClassA, +ClassB, -Path) is nondet.
%
% Finds all paths from ClassA upwards to ClassB.
% Fails if there is no path upwards from ClassA to ClassB.
%
% @param ClassA The starting class
% @param ClassB The goal class
% @param Path The path from ClassA to ClassB
%
path_up(ClassA, ClassA, []).
path_up(ClassA, ClassB, [SuperClass|Path]) :-
    findall(SC, subclass_of(ClassA, SC), SuperClasses),
    member(SuperClass, SuperClasses),
    path_up(SuperClass, ClassB, Path).

%% path_down(+ClassA, +ClassB, -Path) is nondet.
%
% Finds all paths from ClassA downwards to ClassB.
% Fails if there is no path downwards from ClassA to ClassB.
%
% @param ClassA The starting class
% @param ClassB The goal class
% @param Path The path from ClassA to ClassB
%
path_down(ClassB, ClassB, []).
path_down(ClassA, ClassB, [SubClass|Path]) :-
    direct_subclasses(ClassA, SubClasses),
    member(SubClass, SubClasses),
    path_down(SubClass, ClassB, Path).

find_path_down([UPa|_], B, [UPa|D]) :-
    path_down(UPa, B, D).
find_path_down([UPa|Up], B, [UPa|D]) :-
    \+ path_down(UPa, B, D),
    find_path_down(Up, B, D).
find_path_down([], _, []).

%% superclasses(+Class, -SuperClasses) is det.
%
% Finds all direct and indirect superclasses of the given class.
% A superclass is any class that the given class is a subclass of.
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param SuperClasses A list of all direct and indirect superclasses, ordered from most specific to least specific.
%
superclasses(Class, SuperClasses) :-
    findall(SuperClass, (
        subclass_of(Class, SuperClass),
        \+ is_bnode(SuperClass)
    ), DirectSuperClasses),
    findall(IndirectSuperClass, (
        member(DirectSuperClass, DirectSuperClasses),
        superclasses(DirectSuperClass, IndirectSuperClasses),
        member(IndirectSuperClass, IndirectSuperClasses),
        \+ is_bnode(IndirectSuperClass)
    ), IndirectSuperClasses),
    append(DirectSuperClasses, IndirectSuperClasses, AllSuperClasses),
    list_to_set(AllSuperClasses, SuperClasses).

%% subclasses(+Class, -SubClasses) is det.
%
% Finds all direct and indirect subclasses of the given class.
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param SubClasses A list of all direct and indirect subclasses
%
subclasses(Class, SubClasses) :-
    setof(SubClass, (
        subclass_of(SubClass, Class),
        \+ is_bnode(SubClass)
    ), SubClasses).

%% direct_superclasses(+Class, -DirectSuperClasses) is det.
%
% Finds all direct superclasses of the given class.
%
% @param Class The IRI or abbreviated name of the class.
% @param DirectSuperClasses A list of all direct superclasses of the given class.
%
direct_superclasses(Class, DirectSuperClasses) :-
    findall(SuperClass, subclass_of(Class, SuperClass), DirectSuperClasses).

%% direct_subclasses(+Class, -DirectSubClasses) is det.
%
% Finds all direct subclasses of the given class.
%
% @param Class The IRI or abbreviated name of the class.
% @param DirectSubClasses A list of all direct subclasses of the given class.
%
direct_subclasses(Class, DirectSubClasses) :-
    findall(SubClass, (
        subclass_of(SubClass, Class),
        (direct_superclasses(SubClass, SuperClasses), member(Class, SuperClasses))
    ), DirectSubClasses).

% Define a predicate that finds the shortest path between two classes
shortest_path(Source, Destination, Path) :-
    % Initialize the queue with the source class and mark it as visited
    bfs([(0, Source, [])], [Source], Destination, Path).

% Define the BFS algorithm
bfs([(Distance, Current, Path) | Queue], _, Current, Path) :-
    % If the destination is found, return the path
    ros_warn('Found path: ~w', [Path]),
    reverse([Current | Path], Path).
bfs([(Distance, Current, Path) | Queue], Visited, Destination, Path) :-
    ros_warn('2 Distance: ~w, Current: ~w, Path: ~w, Queue: ~w, Visited: ~w', [Distance, Current, Path, Queue, Visited]),
    % Explore the subclasses of the current class
    direct_subclasses(Current, Subclasses),
    ros_warn('2 subclasses of ~w: ~w', [Current, Subclasses]),
    !.
    % addNeighbors(Subclasses, Distance, Current, Path, Queue1, NewVisited, FinalVisited),
    % % Continue the BFS algorithm with the new queue and visited set
    % bfs(Queue1, FinalVisited, Destination, Path).
bfs([(Distance, Current, [])], Visited, Destination, Path) :-
    % If the destination is not found yet, but the queue is empty, go up to the superclasses again
    direct_superclasses(Current, Superclasses),
    ros_warn('1 Superclasses of ~w: ~w', [Current, Superclasses]),
    addNeighbors(Superclasses, Distance, Current, Path, Queue, Visited, NewVisited),
    ros_warn('New queue: ~w', [Queue1]),
    bfs(Queue, NewVisited, Destination, Path).

% Define a helper predicate that adds the neighbors of a class to the queue
addNeighbors([], _, _, _, Queue, Visited, Visited).
addNeighbors([Neighbor | Neighbors], Distance, Current, Path, Queue, Visited, NewVisited) :-
    % Add the neighbor to the queue if it hasn't been visited yet
    not(member(Neighbor, Visited)),
    NewDistance is Distance + 1,
    append(Path, [Current], NewPath),
    append(Queue, [(NewDistance, Neighbor, NewPath)], NewQueue),
    addNeighbors(Neighbors, Distance, Current, Path, NewQueue, [Neighbor | Visited], NewVisited).
addNeighbors([Neighbor | Neighbors], Distance, Current, Path, Queue, Visited, NewVisited) :-
    % Skip the neighbor if it has already been visited
    member(Neighbor, Visited),
    addNeighbors(Neighbors, Distance, Current, Path, Queue, Visited, NewVisited).

%% is_bnode(+IRI) is det.
%
% True if the given IRI is a blank node.
%
% @param IRI The IRI to check
%
is_bnode(IRI) :-
    split_iri(IRI, _, ClassIdentifier),
    rdf_is_bnode(ClassIdentifier).