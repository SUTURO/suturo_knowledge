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
    kb_project(is_type(Object, Type)),
    universal_scope(Scope),
    tf_set_pose(Object, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope),
    !.

