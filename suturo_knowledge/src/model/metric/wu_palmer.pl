%% The WuPalmer module contains predicates that can calculate the similarity between two classes of an ontology.
:- module(wu_palmer,
	  [
        wu_palmer_similarity/3
	  ]).

:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- use_module(library('util/util'),
	      [
		  ros_warn/2
	      ]).

%% wu_palmer_similarity(+Class1, +Class2, -Similarity) is det
%
% Calculates the Wu-Palmer similarity between two classes
%
% @param Class1 One of the two classes
% @param Class2 Second of the two classes
% @param Similarity The similarty measure between 0 (not similar) and 1 (most similar)
%
wu_palmer_similarity(Class1, Class2, Similarity) :-
    % Get the least common subsumer (LCS)
    ros_warn("Test 1"),
    lcs(Class1, Class2, LCS),
    ros_warn('Test 2 ~w', [LCS]),
    % Calculate the depth of the LCS and both classes
    depth(Class1, LCS, Depth1),
    depth(Class2, LCS, Depth2),
    depth(LCS, LCS, DepthLCS),
    % Calculate the Wu-Palmer similarity measure
    Similarity is (2 * DepthLCS) / (Depth1 + Depth2 + (2 * DepthLCS)).

% Least common subsumer (LCS) of two classes
lcs(Class1, Class2, LCS) :-
    % Get the superclasses of both classes
    findall(SuperClass1, subclass_of(Class1, SuperClass1), SuperClasses1),
    findall(SuperClass2, subclass_of(Class2, SuperClass2), SuperClasses2),
    % Find the least common subsumer (LCS)
    find_lcs(SuperClasses1, SuperClasses2, LCS).

% Helper predicate to find the least common subsumer (LCS)
find_lcs([Class|_], Classes, Class) :-
    member(Class, Classes),
    !.
find_lcs([_|Classes1], Classes2, LCS) :-
    find_lcs(Classes1, Classes2, LCS).

% base case: two classes are the same
depth(Class, Class, 0).
% recursive case
depth(ClassA, ClassB, Depth) :-
    ros_warn('Test depth 1 ~w', [ClassB]),
    % find all super classes of ClassA
    findall(SuperClassA, subclass_of(ClassA, SuperClassA), SuperClassesA),
    ros_warn('Test depth 2 ~w~n', [SuperClassesA]),
    % find all super classes of ClassB
    findall(SuperClassB, subclass_of(ClassB, SuperClassB), SuperClassesB),
    ros_warn('Test depth 2 ~w~n', [SuperClassesB]),
    % get the intersection of super classes
    intersection(SuperClassesA, SuperClassesB, CommonSuperClasses),
    ros_warn('Test depth 3 ~w~n', [CommonSuperClasses]),
    % find the minimum depth among all common super classes
    findall(D, (
        member(CommonSuperClass, CommonSuperClasses),
        depth(ClassA, CommonSuperClass, D1),
        depth(ClassB, CommonSuperClass, D2),
        D is D1 + D2
    ), Depths),
    min_list(Depths, Depth).