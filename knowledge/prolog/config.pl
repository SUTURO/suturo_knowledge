:- module(config,
    [
      offsets/1,
      threshold_for_group/1,
      threshold_surface/2,
      urdf_surface_prefix/1,
      allowed_class_distance/1,
      min_class_confidence/1, 
      context_speech_sort_by_class/4,
      context_speech_sort_by_color/3,
      context_speech_sort_by_size/3,
      context_speech_new_class/1
    ]).


:- rdf_meta
    offests(?),
    allowed_class_distance(?),
    context_speech_sort_by_class(-,-,-,?),
    context_speech_sort_by_color(-,-,?),
    context_speech_sort_by_size(-,-,?),
    context_speech_new_class(?).


:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).


offsets(Offset) :-
    Offset = [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, 0.35, 0.35].

% max physical Distance between objects for them to be in a group
threshold_for_group(Threshold) :-
    Threshold = 0.15.

threshold_surface(ThresholdAbove, ThresholdBelow) :-
    ThresholdAbove = 0.25,
    ThresholdBelow = -0.05.

urdf_surface_prefix(Prefix) :-
    Prefix = 'iai_kitchen/'.

%% Distance is the maximum Distance (rdf_shortest_path) to another Object
%% where the Object should still be sorted by class rather than other properties
%% like Color or Size.
allowed_class_distance(Distance) :-
    Distance = 5.

%% Minimum Confidence where the perceived Object class should still be stored.
%% Classes with lower confidence get 'Other' as fallback.
min_class_confidence(Confidence) :-
    Confidence = 0.5.

%% Context is the Speech, the Robot should hold depending on the distance of the
%% next Objects Class.
context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance =< 1, %% exactly the same class
    string_concat('I will put this ', Object, Part1),
    string_concat('to the other', SimilarObject, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance =< 3, %% direct superclass or child of the same super class
    Distance >=2,
    string_concat('I will put this ', Object, Part1),
    string_concat('to the similar ', SimilarObject, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_class(Object, SimilarObject, Distance, Context) :-
    Distance >= 4,
    string_concat('I will put this ', Object, Part1),
    string_concat('to the somehow similar ', SimilarObject, Part2),
    string_concat(Part1, Part2, Context).

context_speech_sort_by_color(Object, SimilarObject, Context) :-
    rdf_has(Object, hsr_objects:'colour', ObjColor),
    rdf_has(SimilarObject, hsr_objects:'colour', SimilarColor),
    string_concat('I will put this ', ObjColor, Part1),
    string_concat(Part1, ' Object to the other ', Part2),
    string_concat(Part2, SimilarColor, Part3),
    string_concat(Part3, ' object.', Context).

context_speech_sort_by_size(Object, SimilarObject, Context) :-
    rdf_has(Object, hsr_objects:'size', ObjSize),
    rdf_has(SimilarObject, hsr_objects:'size', SimilarSize),
    string_concat('I will put this ', ObjSize, Part1),
    string_concat(Part1, ' Object to the other ', Part2),
    string_concat(Part2, SimilarSize, Part3),
    string_concat(Part3, ' object.', Context).

context_speech_new_class(Context):-
    Context = 'I will create a new group for this'.