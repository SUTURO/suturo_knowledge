%% Semantic Similarity
% The Semantic Similarity module contains predicates that can calculate the similarity between two classes of an ontology.
% It further contains predicates to find superclasses and subclasses of a class and calculate the depth of a class in the ontology hierarchy.
:- module(semantic_similarity,
	  [
        wu_palmer_similarity(r,r,-),
        lcs(r,r,-),
        superclasses(r,-),
        subclasses(r,-),
        direct_superclasses(r,-),
        direct_subclasses(r,-),
        class_depths(r,-),
        min_class_depth(r,-),
        max_class_depth(r,-)
	  ]).

:- use_module(library('semweb/rdf_db')).

:- use_module(library('util/util'),
	      [
		  split_iri/3,
		  is_bnode/1,
		  ros_warn/2
	      ]).

%% wu_palmer_similarity(+ClassA, +ClassB, -Similarity) is det.
%
% Calculates the Wu-Palmer similarity between two classes.
% The similarity can be 0 < similarity <= 1.
% The similarity can never be zero because the depth of the LCS is never zero (the depth of the root of taxonomy is one). 
%
% @param ClassA One of the two classes
% @param ClassB Second of the two classes
% @param Similarity The similarty measure between 0 (not similar) and 1 (most similar)
%
wu_palmer_similarity(ClassA, ClassA, 1) :- 
    !.
wu_palmer_similarity(ClassA, ClassB, Similarity) :-
    % Get the least common subsumer (LCS)
    lcs(ClassA, ClassB, LCS),
    % Calculate the depth of the LCS and both classes
    min_class_depth(LCS, DepthLCS),
    min_class_depth(ClassA, DepthClassA),
    min_class_depth(ClassB, DepthClassB),
    % Calculate the Wu-Palmer similarity measure
    Similarity is (2 * DepthLCS) / (DepthClassA + DepthClassB).

%% lcs(+ClassA, +ClassB, -LCS) is nondet.
%
% Least common subsumer/superclass (LCS) of two classes.
% The LCS is the most specific class that is a superclass of both classes.
% If there are multiple LCSs, the LCS with the lowest depth in the ontology hierarchy is returned.
%
% @param ClassA One of the two classes
% @param ClassB Second of the two classes
% @param LCS The least common subsumer of the two classes
%
lcs(ClassA, ClassB, LCS) :-
    superclasses(ClassA, SuperClassesA),
    superclasses(ClassB, SuperClassesB),
    intersection(SuperClassesA, SuperClassesB, CommonSuperClasses),
    member(LCS, CommonSuperClasses),
    \+ (
        member(OtherCommonSuperClass, CommonSuperClasses),
        member(OtherCommonSuperClass, SuperClassesA),
        member(OtherCommonSuperClass, SuperClassesB),
        OtherCommonSuperClass \= LCS,
        superclasses(OtherCommonSuperClass, OtherSuperClasses),
        member(LCS, OtherSuperClasses)
    ).

%% superclasses(+Class, -SuperClasses) is det.
%
% Finds all direct and indirect superclasses of the given class.
% A superclass is any class that the given class is a subclass of.
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param SuperClasses A list of all direct and indirect superclasses
%
superclasses(Class, SuperClasses) :-
    setof(SuperClass, find_superclass(Class, SuperClass), SuperClasses).

%% find_superclass(+Class, -SuperClass) is nondet.
%
% Helper predicate to find all direct and indirect superclasses of the given class.
% A superclass is any class that the given class is a subclass of.
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param SuperClass A superclass of the given class.
%
find_superclass(Class, SuperClass) :-
    direct_superclasses(Class, DirectSuperClasses),
    member(DirectSuperClass, DirectSuperClasses),
    (   DirectSuperClass = SuperClass
    ;   find_superclass(DirectSuperClass, SuperClass)
    ).

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
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param DirectSuperClasses A list of all direct superclasses of the given class.
%
direct_superclasses(Class, DirectSuperClasses) :-
    findall(SuperClass, (
        subclass_of(Class, SuperClass),
        \+ is_bnode(SuperClass)
    ), DirectSuperClasses).

%% direct_subclasses(+Class, -DirectSubClasses) is det.
%
% Finds all direct subclasses of the given class.,
% Blank nodes are excluded from the results.
%
% @param Class The IRI or abbreviated name of the class.
% @param DirectSubClasses A list of all direct subclasses of the given class.
%
direct_subclasses(Class, DirectSubClasses) :-
    findall(SubClass, (
        subclass_of(SubClass, Class),
        (direct_superclasses(SubClass, SuperClasses), member(Class, SuperClasses))
    ), DirectSubClasses).

%% class_depths(+Class, -Depths) is det.
%
% Finds all depth levels of the given class in the ontology hierarchy.
% The depth of a class is defined as the number of superclass links from the class to the root of the ontology.
% Each depth level is calculated by traversing the hierarchy from the class upwards to the root.
% The root of the ontology is a class that has no superclasses. This is usually dul:'Entity' with a depth of 1.
%
% @param Class The IRI or abbreviated name of the class.
% @param Depths A list of all depth levels of the given class, each represented by an integer.
%
class_depths(Class, Depths) :-
    setof(Depth, find_class_depth(Class, [], Depth), Depths).

%% find_class_depth(+Class, +Visited, -Depth) is nondet.
%
% Finds a depth level of the given class in the ontology hierarchy by traversing the hierarchy from the class upwards to the root.
% The depth is represented by an integer.
% To avoid infinite loops caused by cycles in the hierarchy, it keeps track of the visited classes.
%
% @param Class The IRI or abbreviated name of the class.
% @param Visited A list of classes that have already been visited during the traversal.
% @param Depth The depth level of the given class.
%
find_class_depth(Class, Visited, Depth) :-
    direct_superclasses(Class, DirectSuperClasses),
    exclude(member(Visited), DirectSuperClasses, UnvisitedSuperClasses),
    (
        UnvisitedSuperClasses = [] -> Depth = 1;
        (
            member(SuperClass, UnvisitedSuperClasses),
            find_class_depth(SuperClass, [Class|Visited], SuperClassDepth),
            Depth is SuperClassDepth + 1
        )
    ).

%% min_class_depth(+Class, -Depth) is det.
%
% Finds the minimum depth level of the given class in the ontology hierarchy.
%
% @param Class The IRI or abbreviated name of the class.
% @param Depth The minimum depth level of the given class.
%
min_class_depth(Class, Depth) :-
    class_depths(Class, Depths),
    min_list(Depths, Depth).

%% max_class_depth(+Class, -Depth) is det.
%
% Finds the maximum depth level of the given class in the ontology hierarchy.
%
% @param Class The IRI or abbreviated name of the class.
% @param Depth The maximum depth level of the given class.
%
max_class_depth(Class, Depth) :-
    class_depths(Class, Depths),
    max_list(Depths, Depth).