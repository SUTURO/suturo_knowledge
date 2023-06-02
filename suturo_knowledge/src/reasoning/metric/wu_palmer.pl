%% The WuPalmer module contains predicates that can calculate the similarity between two classes of an ontology.
:- module(wu_palmer,
	  [
        wu_palmer_similarity(r,r,-),
        path_up(r,r,-),
        path_down(r,r,-),
        path(r,r,-),
        superclasses(r,-),
        subclasses(r,-),
        direct_superclasses(r,-),
        direct_subclasses(r,-)
	  ]).

:- use_module(library('semweb/rdf_db')).

:- use_module(library('util/util'),
	      [
		  split_iri/3,
		  ros_warn/2
	      ]).

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

%% path(+A, +B, -Path).
%
% Compute path between classes A and B
%
% @param A      OWL class
% @param B      OWL class
% @param Path  List of elements along the path between A and B
%
path(A, B, [Path]) :-
    subclass_of(A, B),
    path_up(A, B, Path).
path(A, B, [Path]) :-
    subclass_of(B, A),
    path_down(A, B, Path).
path(A, B, [A|Path]) :-
    % go upwards, and for all paths check if you have a direct way down to the goal
    path_up(A, dul:'Entity', U),
    find_path_down(U, B, Path).
  
find_path_down([UPa|_], B, [UPa|D]) :-
    path_down(UPa, B, D).
find_path_down([UPa|Up], B, [UPa|D]) :-
    \+ path_down(UPa, B, D),
    find_path_down(Up, B, D).
find_path_down([], _, []).

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
    direct_superclasses(ClassA, SuperClasses),
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

%% is_bnode(+IRI) is det.
%
% True if the given IRI is a blank node.
%
% @param IRI The IRI to check
%
is_bnode(IRI) :-
    split_iri(IRI, _, ClassIdentifier),
    rdf_is_bnode(ClassIdentifier).