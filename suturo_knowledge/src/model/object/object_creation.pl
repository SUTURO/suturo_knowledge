:- module(object_creation,
	[
		cereal_box/1,
        create_object/3
	]).

cereal_box(Entity) ?>
    has_type(Entity, soma:'CerealBox').

cereal_box(Entity) +>
    new_iri(Entity, soma:'CerealBox'),
	has_type(Entity, soma:'CerealBox').

create_object(Obj, Type, [Frame, [X,Y,Z], [RX,RY,RZ,RW]]) :-
    kb_project(new_iri(Obj, Type)),
    kb_project(has_type(Obj, Type)),
    universal_scope(Scope),
    tf_set_pose(Obj, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope).

 



