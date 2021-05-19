:- module(config,
    [
      offsets/1,
      threshold_for_group/1,
      threshold_surface/2,
      min_space_between_objects/1,
      urdf_surface_prefix/1,
      allowed_class_distance/1,
      min_class_confidence/1, 
      min_shape_confidence/1,
      min_color_confidence/1,
      max_shelf_capacity/1,
      context_speech_sort_by_class/4,
      context_speech_sort_by_color/3,
      context_speech_sort_by_size/3,
      context_speech_new_class/1,
      context_speech_basket/1,
      get_urdf_id/1,
      get_urdf_origin/1
    ]).


:- rdf_meta
    offests(?),
    allowed_class_distance(?),
    context_speech_sort_by_class(-,-,-,?),
    context_speech_sort_by_color(-,-,?),
    context_speech_sort_by_size(-,-,?),
    context_speech_new_class(?).


:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).


offsets(Offset) :-
    Offset = [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, -0.35, 0.35, -0.4, 0.4, -0.45, 0.45, -0.5, 0.5, -0.55, 0.55, -0.6, 0.6, -0.65, 0.65, -0.7, 0.7, -0.75, 0.75, -0.8, 0.8].

% max physical Distance between objects for them to be in a group
threshold_for_group(Threshold) :-
    Threshold = 0.15.

threshold_surface(ThresholdAbove, ThresholdBelow) :-
    ThresholdAbove = 0.25,
    ThresholdBelow = -0.05.

min_space_between_objects(Meters) :-
    Meters = 0.05.

urdf_surface_prefix(Prefix) :-
    Prefix = 'iai_kitchen/'.

%% Distance is the maximum Distance (rdf_shortest_path) to another Object
%% where the Object should still be sorted by class rather than other properties
%% like Color or Size.
allowed_class_distance(Distance) :-
    Distance = 7.

%% Minimum Confidence where the perceived Object class should still be stored.
%% Classes with lower confidence get 'Other' as fallback.
min_class_confidence(Confidence) :-
    Confidence = 0.5.

min_shape_confidence(Confidence) :-
    Confidence = 0.5.

min_color_confidence(Confidence) :-
    Confidence = 0.5.

max_shelf_capacity(Capacity) :-
    Capacity = 4.

%% Context is the Speech, the Robot should hold depending on the distance of the
%% next Objects Class.
context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance =< 1, %% exactly the same class
    object_classname(Object, ObjectClass),
    object_classname(SimilarObject, SimilarObjectClass),
    string_concat('I will put this ', ObjectClass, Part1),
    string_concat(' to the other ', SimilarObjectClass, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance =< 3, %% direct superclass or child of the same super class
    Distance >=2,
    object_classname(Object, ObjectClass),
    object_classname(SimilarObject, SimilarObjectClass),
    string_concat('I will put this ', ObjectClass, Part1),
    string_concat(' to the similar ', SimilarObjectClass, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance >= 4,
    object_classname(Object, ObjectClass),
    object_classname(SimilarObject, SimilarObjectClass),
    string_concat('I will put this ', ObjectClass, Part1),
    string_concat(' to the somehow similar ', SimilarObjectClass, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_color(Object, SimilarObject, Context) :-
    triple(Object, hsr_objects:'colour', ObjColor),
    triple(SimilarObject, hsr_objects:'colour', SimilarColor),
    string_concat('I will put this ', ObjColor, Part1),
    string_concat(Part1, ' Object to the other ', Part2),
    string_concat(Part2, SimilarColor, Part3),
    string_concat(Part3, ' object.', Context).

context_speech_sort_by_size(Object, SimilarObject, Context) :-
    triple(Object, hsr_objects:'size', ObjSize),
    triple(SimilarObject, hsr_objects:'size', SimilarSize),
    string_concat('I will put this ', ObjSize, Part1),
    string_concat(Part1, ' Object to the other ', Part2),
    string_concat(Part2, SimilarSize, Part3),
    string_concat(Part3, ' object.', Context).

% Object is an actual Object, where
% Classname is the Name of its class without the hsr_objects: in front of it.
object_classname(Object, Classname) :-
    has_type(Object, Type),
    split_string(Type, "#", "", [_,ClassnameString]),
    atom_string(Classname, ClassnameString).

context_speech_new_class(Context) :-
    Context = "I will create a new group for this".

context_speech_basket(Context) :-
    Context = "I will put this in the Basket".


get_urdf_id(URDF) :-
    URDF = arena.

get_urdf_origin(Origin) :-
    Origin = map.

