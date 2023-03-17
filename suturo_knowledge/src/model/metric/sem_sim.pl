:- use_module(library('semweb/rdfs')).

% Load the ontology
load_ontology(File) :-
    owl_parse(File),
    owl_init_reasoner.

% Calculate the semantic similarity between two entities
semantic_similarity(Entity1, Entity2, Similarity) :-
    % Get the most informative common ancestor (MICA) of the two entities
    mica(Entity1, Entity2, Mica),
    % Calculate the depth similarity
    depth_similarity(Entity1, Entity2, Mica, DepthSimilarity),
    % Calculate the path similarity
    path_similarity(Entity1, Entity2, PathSimilarity),
    % Combine the two similarity scores using a weighted average
    Similarity is 0.5 * DepthSimilarity + 0.5 * PathSimilarity.

% Get the most informative common ancestor (MICA) of two entities
mica(Entity1, Entity2, Mica) :-
    % Get the set of all ancestors of Entity1
    get_ancestors(Entity1, Ancestors1),
    % Get the set of all ancestors of Entity2
    get_ancestors(Entity2, Ancestors2),
    % Find the intersection of the two sets of ancestors
    intersection(Ancestors1, Ancestors2, Intersection),
    % Find the most informative common ancestor (MICA) of the two entities
    find_mica(Intersection, Mica).

% Get the set of all ancestors of an entity
get_ancestors(Entity, Ancestors) :-
    % Find all superclasses of the entity
    owl_subclass_of(Entity, Ancestor),
    % Recursively find all ancestors of the entity
    (   Ancestor = owl:'Thing'
    ->  Ancestors = [Ancestor]
    ;   get_ancestors(Ancestor, Ancestors0),
        append([Ancestor], Ancestors0, Ancestors)
    ).

% Find the most informative common ancestor (MICA) of a set of entities
find_mica([Entity], Entity) :- !.
find_mica([Entity|Entities], Mica) :-
    % Find the set of all ancestors of the first entity
    get_ancestors(Entity, Ancestors),
    % Find the set of all common ancestors of the first entity and the rest of the entities
    find_common_ancestors(Entities, Ancestors, CommonAncestors),
    % Find the most informative ancestor (MIA) of the set of common ancestors
    find_mia(CommonAncestors, Mia),
    % Recursively find the most informative common ancestor (MICA) of the rest of the entities
    find_mica(Entities, Mica0),
    % Find the most informative ancestor (MIA) of the MICA and the MIA
    find_mia([Mica0, Mia], Mica).

% Find the set of all common ancestors of a set of entities
find_common_ancestors([], CommonAncestors, CommonAncestors).
find_common_ancestors([Entity|Entities], CommonAncestors0, CommonAncestors) :-
    % Find the set of all ancestors of the entity
    get_ancestors(Entity, Ancestors),
    % Find the intersection of the set of ancestors and the current set of common ancestors
    intersection(Ancestors, CommonAncestors0, Intersection),
    % Add the intersection to the current set of common ancestors
    append(Intersection, CommonAncestors0, CommonAncestors1),
    % Recursively find the common ancestors of the rest of the entities
    find_common_ancestors(Entities, CommonAncestors1, CommonAncestors).

% Find the most informative ancestor (MIA) of a set of entities
find_mia([Entity], Entity) :- !.
find_mia([Entity|Entities], Mia) :-
    % Find the set of all ancestors of the entity
    get_ancestors(Entity, Ancestors),
    % Find the maximum information content (IC) of the ancestors
    find_max_ic(Ancestors, MaxIc),
    % Recursively find the most informative ancestor (MIA) of the rest of the entities
    find_mia(Entities, Mia0),
    % Find the most informative ancestor (MIA) of the MIA and the entity
    find_mia([Mia0, Entity], Mia1),
    % Find the maximum information content (IC) of the MIA and the maximum IC
    find_max_ic([Mia1, MaxIc], Mia).

% Find the maximum information content (IC) of a set of entities
find_max_ic(Entities, MaxIc) :-
    % Find the information content (IC) of each entity
    findall(Ic, (member(Entity, Entities), ic(Entity, Ic)), Ics),
    % Find the maximum IC
    max_list(Ics, MaxIc).

% Calculate the information content (IC) of an entity
ic(Entity, Ic) :-
    % Find the number of instances of the entity
    owl_individual_of(Instance, Entity),
    findall(Instance, owl_individual_of(Instance, Entity), Instances),
    length(Instances, N),
    % Find the total number of instances in the ontology
    findall(Instance, owl_individual_of(Instance, owl:'Thing'), AllInstances),
    length(AllInstances, M),
    % Calculate the probability of an instance being of the entity
    P is N / M,
    % Calculate the information content (IC) of the entity
    Ic is -log(P).

% Calculate the depth similarity between two entities
depth_similarity(Entity1, Entity2, Mica, Similarity) :-
    % Find the depth of the MICA
    depth(Mica, Depth),
    % Find the depth of Entity1
    depth(Entity1, Depth1),
    % Find the depth of Entity2
    depth(Entity2, Depth2),
    % Calculate the depth similarity
    Similarity is 2 * Depth / (Depth1 + Depth2 + 2 * Depth).

% Calculate the path similarity between two entities
path_similarity(Entity1, Entity2, Similarity) :-
    % Find the shortest path between the two entities
    shortest_path(Entity1, Entity2, Path),
    % Calculate the path similarity
    length(Path, L),
    Similarity is 2 / (L + 1).