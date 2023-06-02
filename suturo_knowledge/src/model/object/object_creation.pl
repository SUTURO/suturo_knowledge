:- module(object_creation,
	[
		create_object(-,r,+),
	    create_object(-,r,+,+)
	]).

:- rdf_meta(shape_class(+,r)).

%% create_object(-Object, +Type, +PoseStamped) is det.
%
% Create an object of type Type at the given PoseStamped.
%
% @param Object The object iri that is created
% @param Type The type of the object (full iri) TODO: add ability to use namespace short form
% @param PoseStamped The pose of the object
%
create_object(Object, Type, PoseStamped) :-
    create_object(Object, Type, PoseStamped, []).

%% create_object(-Object, +Type, +PoseStampedm, +Options) is det.
%
% see create_object/3 for the simple documentation.
% This predicate also processes options:
% - shape(ShapeTerm)
% - data_source(DataSource) (should be either perception or semantic_map, as described in [object_model.md](../../../object_model.md)
% - confidence_value(ConfidenceValue) (Confidence is between 0 and 1)
create_object(Object, Type, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Options) :-
    from_current_scope(Scope),
    kb_project(is_type(Object, Type), Scope),
    tf_set_pose(Object, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope),
    option(shape(Shape), Options, none),
    assert_shape(Object, Shape, Scope),
    option(data_source(DataSource), Options, perception),
    kb_project(triple(Object, suturo:hasDataSource, DataSource)),
	option(confidence_value(ConfidenceValue), Options, 1),
    kb_project(triple(Object, suturo:hasConfidenceValue, ConfidenceValue)),
    kb_project((new_iri(HandleState),
				triple(Object, suturo:hasHandleState, HandleState),
				triple(HandleState,suturo:handled,false))),
    !.

assert_shape(_Object, none, _Scope) :- !.
assert_shape(Object, ShapeTerm, Scope) :-
    shape_class(ShapeTerm, Class),
    kb_project((is_type(Shape, soma:'Shape'),
		is_type(SR, Class),
		triple(Object,soma:hasShape,Shape),
		triple(Shape,dul:hasRegion,SR)), Scope),
    assert_shape_region(SR, ShapeTerm, Scope),
    !.

shape_class(box(_,_,_), soma:'BoxShape'):- !.
shape_class(cylinder(_,_), soma:'CylinderShape'):- !.
shape_class(mesh(_,[_,_,_]), soma:'MeshShape'):- !.
shape_class(sphere(_), soma:'SphereShape'):- !.

assert_shape_region(SR, mesh(File, [1,1,1]), Scope) :-
    kb_project(triple(SR, soma:hasFilePath, File), Scope),
    !.

assert_shape_region(SR, box(X,Y,Z), Scope) :-
	kb_project((triple(SR, soma:hasDepth,  X),
		    triple(SR, soma:hasWidth,  Y),
		    triple(SR, soma:hasHeight, Z)), Scope),
	!.

assert_shape_region(SR, cylinder(Radius,Length), Scope) :-
    kb_project((triple(SR, soma:hasLength, Length),
		triple(SR, soma:hasRadius, Radius)), Scope),
	!.

assert_shape_region(SR, sphere(Radius), Scope) :-
	kb_project(triple(SR, soma:hasRadius, Radius), Scope),
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
