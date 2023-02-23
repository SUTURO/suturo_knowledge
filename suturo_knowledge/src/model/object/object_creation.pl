:- module(object_creation,
	[
        create_object/3
	]).

%% create_object(-Object, +Type, +PoseStamped) is det.
%
% Create an object of type Type at the given PoseStamped.
%
% @param Object The object iri that is created
% @param Type The type of the object (full iri) TODO: add ability to use namespace short form
% @param PoseStamped The pose of the object
%
create_object(Object, Type, [Frame, [X,Y,Z], [RX,RY,RZ,RW]]) :-
    from_current_scope(Scope),
    kb_project(is_type(Object, Type), Scope),
    kb_project(triple(Object, suturo:isManagedBy, suturo_knowledge), Scope),
    tf_set_pose(Object, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope),
    kb_project(is_type(Shape, soma:'Shape'), Scope),
    kb_project(triple(Object,soma:hasShape,Shape), Scope),
    kb_project(is_type(SR, soma:'BoxShape'), Scope),
    kb_project(triple(Shape,dul:hasRegion,SR), Scope),
    kb_project(triple(SR, soma:hasDepth,  0.1), Scope),
    kb_project(triple(SR, soma:hasWidth,  0.2), Scope),
    kb_project(triple(SR, soma:hasHeight, 0.3), Scope),
    !.

%% from_current_scope(-Scope) is det.
%
% The scope of facts that are true from now until infinity.
%
% @param Scope A scope dictionary.
%
from_current_scope(dict{
		       time: dict{
				 since: =(double(Now)),
				 until: =(double('Infinity'))
	}
}) :- get_time(Now).
