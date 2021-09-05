:- module(object_info, [
    is_suturo_object/1,
    hsr_existing_objects/1,
    hsr_forget_object/1,
    forget_objects_on_surface_/1,
    place_object/1
    ]).


is_suturo_object(Object) :-
    has_type(Object, hsr_objects:'EnduringThingLocalized').


% returns a list of all the Objects know to the Knowledgebase
hsr_existing_objects(Objects) :-
    findall(PO, (
        ask(has_type(PO, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject'))
    ), POs),
    findall(Obj, (
        has_type(Obj,_),
        triple(Obj, hsr_objects:'supportable', true),
        member(Obj, POs)
    ), Objs),
    list_to_set(Objs,Objects).

%% hsr_forget_object(Object) is det.
%
% Forget a specific Object.
%
% @param Object the object to forget.
hsr_forget_object(Object) :-
    forall(triple(Object,X,Y), tripledb_forget(Object,X,Y)).
    % TODO we need to stop publishing the tf and marker

%% forget_objects_on_surface_(SurfaceLink) is det.
%
% Forget all objects on surface.
%
% @param Object the object to forget.
forget_objects_on_surface_(SurfaceLink) :-
    objects_on_surface(Objs,SurfaceLink),
    member(Obj,Objs),
    hsr_forget_object(Obj).


%% place_object(Object) is ?
%
% Finds the surface an object was seen on. When there is no surface supporting the object and
% the center point of the object < 0.5 the object is placed on the ground.
% Otherwise the query resolves to false.
% @param Object the object to find the surface on.
place_object(Object):-
    object_supported_by_surface(Object, Surface),
    assert_object_on(Object,Surface).