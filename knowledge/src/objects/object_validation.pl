:- module(object_validation, [
    validate_confidence/3,
    object_size_ok/1,
    object_type_handling/3
]).

%%%%%%%%%% TODO what was the purpose of this code? %%%%%%%%%%
validate_confidence(class, Is, Should) :-
    var(Is),
    min_class_confidence(Should).

validate_confidence(shape, Is, Should) :-
    var(Is),
    min_shape_confidence(Should).

validate_confidence(color, Is, Should) :-
    var(Is),
    min_color_confidence(Should).

validate_confidence(_, Is, Should) :-
    Should = Is.

object_size_ok([Width,Depth,Height]):-
    Width > 0.01,
    Depth > 0.01,
    Height > 0.01,
    Width < 0.6,
    Depth < 0.6,
    Height < 0.6.

% Determines the ObjectType the Object is saved as, it is PerceivedObjectType when the Conf is high enough Otherwise it is Other
%object_type_handling(PerceivedObjectType, TypeConfidence, ObjectType) :-
%    min_class_confidence(MinConf),
%    (   number(TypeConfidence),
%        TypeConfidence >= MinConf
%        -> ObjectType = PerceivedObjectType;
%        ObjectType = 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Other'
%        ).

object_type_handling(PerceivedObjectType, ClassConfidence, ObjectType) :-
    (   min_class_confidence(ClassConfidence)
        -> ObjectType = PerceivedObjectType;
        ObjectType = 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Other'
        ).