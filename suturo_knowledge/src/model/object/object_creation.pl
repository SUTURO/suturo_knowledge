:- module(object_creation,
	[
		create_object(-,r,+),
	    create_object(-,r,+,+)
	]).

:- rdf_meta(shape_class(+,r)).

:- use_module(library('util/util'), 
    [
        from_current_scope/1,
        default_value/2
    ]).

%% create_object(-Object, +Type, +PoseStamped) is det.
%
% Create an object of type Type at the given PoseStamped.
% The data can be retrieved via `object_shape/5`
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
    assert_shape(Object, Shape, Scope, SR),
    option(data_source(DataSource), Options, perception),
    (  DataSource == perception
       % ignore, because maybe object is not above any furniture
    -> ignore(assert_relative_position(Object, Scope))
    ;  true),
    kb_project(triple(Object, suturo:hasDataSource, DataSource), Scope),
	option(confidence_value(ConfidenceValue), Options, 1),
    kb_project(triple(Object, suturo:hasConfidenceValue, ConfidenceValue), Scope),
    kb_project((new_iri(HandleState),
				triple(Object, suturo:hasHandleState, HandleState),
				triple(HandleState,suturo:handled,false)), Scope),
    % doesn't work currently, see https://github.com/knowrob/knowrob/issues/371 for more information.
    %assert_origin(SR, Scope),
    !.

assert_shape(_Object, none, _Scope, _SR) :- !.
assert_shape(Object, ShapeTerm, Scope, SR) :-
    shape_class(ShapeTerm, Class),
    kb_project((is_type(Shape, soma:'Shape'),
		is_type(SR, Class),
		triple(Object,soma:hasShape,Shape),
		triple(Shape,dul:hasRegion,SR)), Scope),
    assert_shape_region(SR, ShapeTerm, Scope),
    !.

assert_origin(SR, Scope) :-
    % set the shape origin, useful so that object_shape/5 returns the correct Pose.
    kb_project(new_iri(Origin, 'http://www.ease-crc.org/ont/SOMA.owl#6DPose'),Scope),
    kb_project(triple(SR, urdf:hasOrigin, Origin), Scope),
    kb_project(triple(Origin, soma:hasPositionVector, [0,0,0]), Scope),
    kb_project(triple(Origin, soma:hasOrientationVector, [0,0,0,1]), Scope).

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

assert_relative_position(Object, Scope) :-
    findall([Furniture, Distance],
            % only objects of the semantic map can be stood on
            (triple(Furniture, suturo:hasDataSource, semantic_map),
             suturo_is_ontop_of(Object, Furniture, Distance)),
            Furnitures),
    maplist(nth0(1), Furnitures, Distances),
    min_list(Distances, MinumumDistance),
    member([Furniture,MinumumDistance], Furnitures),
    kb_project(triple(Object, soma:isOntopOf, Furniture), Scope).

suturo_is_ontop_of(Object, Furniture, Distance) :-
    object_shape_workaround(Furniture, Frame, ShapeTerm, [_, [XX, YY, ZZ], _], _),
    kb_call(is_at(Object, [Frame, [X,Y,Z], _])),
    ShapeTerm = box(DX, DY, _DZ),
    default_value(XX, 0),
    default_value(YY, 0),
    default_value(ZZ, 0),
    % Assuming z is up direction,
    % X and Y have to be above the area of the table.
    % so between center + diameter / 2 and center - diameter / 2.
    % center is at -offset
    X =< -XX + DX/2,
    X >= -XX - DX/2,
    Y =< -YY + DY/2,
    Y >= -YY - DY/2,
    % Z has to be above
    % this code assumes that the frame is at the top center of the furniture.
    Z >= ZZ,
    Distance is Z + ZZ.
