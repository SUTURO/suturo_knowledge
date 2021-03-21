:- module(object_state,
    [
    hsr_existing_objects/1,
    hsr_forget_object/1,
    forget_objects_on_surface_/1,
    place_object/1,
    create_object/9,
    random_id_gen/2,
    validate_confidence/3,
    object_size_ok/1,
    object_type_handling/3,
    set_object_color/3,
    set_color_semantics/2,
    reachability_check/3,
    check_too_small/1,
    republish/0,
    set_test_for_graspable/0
%    test_predicate/0
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(soma, 'https://ease-crc.github.io/soma/owl/current/SOMA.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://www.knowrob.org/kb/knowrob.owl#', [keep(true)]).
:- use_module(library('ros/marker/marker_plugin'), [republish/0]). % Importing marker_plugin

% TODO rdf meta
:- rdf_meta
    create_object(r,?),
	object_at(r,r,r,?),
	object_at_table(?),
	object_of_type(r,?),
	create_object_at(r,r,r,?,-,-),
	hsr_existing_objects(?).

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
    object_supportable_by_surface(Object, Surface),
    assert_object_on(Object,Surface).

%% create_object(PerceivedObjectType, PercTypeConf, Transform, [Width, Depth, Height], 'box', PercShapeConf, Color, PercColorConf, ObjID is nondet.
%
% Validate the perceived params, create an Object instance of the corresponding Ontology,
% write it into the triple store and publish accordingly to ros topics.
% @param PerceivedObjectType the Object Type that perception observed.
% @param PercTypeConf the confidence of the Object Type.
% @param Transform the tranformation of the Object Pose Matrix in the coordinance map.
% @param [Width, Depth, Height] the corresponding dimensions.
% @param 'box' the Object Shape that is fixed on a box shape.
% @param PercShapeConf the confidence of the shape.
% @param Color the observed color of the object.
% @param PercColorConf the confidence of the color.
% @param ObjID the id that will be generated for the Object.
create_object(PerceivedObjectType, PercTypeConf, Transform, [Width, Depth, Height], 'box', PercShapeConf, Color, PercColorConf, ObjID):-
    % TODO refactor into more suitable predicates
    % TODO change that terriple param order
    % TODO improve 'box' param
    % TODO fix the marker_plugin Warnings
    % TODO go over all db writings, where to we actually need a tell ?

    %%% ================ Object validation
    % TODO make this dynamic to constraints
    object_size_ok([Width, Depth, Height]), % Dont add the object when the size is to big/small
    validate_confidence(class, PercTypeConf, TypeConf),
    validate_confidence(shape, PercShapeConf, ShapeConf),
    validate_confidence(color, PercColorConf, ColorConf),
    object_type_handling(PerceivedObjectType, PercTypeConf, ObjectType), % When the PercTypeConf is to low the Type is set to Other, Otherwise ObjectType is the same as PerceivedObjectType
    random_id_gen(6, Result),  % create ID = Type + random id
    atom_concat(ObjectType, '_', ObjectTypeU),
    atom_concat(ObjectTypeU, Result, ObjID),
    % TODO check if the ID is already used

    %%% ================ Object creation
    tell(has_type(ObjID, ObjectType)), % Create Object of type ObjectType           // +1 S=ObjID
    tell(is_physical_object(ObjID)), % write into triple: ont: is-a                 // +1 P=ObjID
    tell(is_at(ObjID,Transform)), % this triggers rosprolog to publish it on tf     // no change! no tell(...) needed? doch needed..
    atom_number(TypeConfidenceAtom, TypeConf),
    tell(triple(ObjID, hsr_objects:'ConfidenceClassValue', TypeConfidenceAtom)), %  // +1 P=ObjID
    ((triple(ObjID, soma:hasShape, Shape), % check if Shape exists                  // +6, 1x P=ObjID, 3x P=ShapeID, 2x P=ShapeRegionID
    triple(Shape,dul:hasRegionReachabilityEnum,ShapeRegion)); % if yes, then check if ShapeRegion exist
    (tell(has_type(Shape, soma:'Shape')), % if either Shape or ShapeRegion does not exist,
    tell(triple(ObjID,soma:hasShape,Shape)), % then create Shape, ShapeRegion
    tell(has_type(ShapeRegion, soma:'ShapeRegion')),
    tell(holds(Shape,dul:hasRegion,ShapeRegion)))),
    Pos = [0,0,0], Rot = [0,0,0,1],
    tell(is_individual(Origin)), % create an individuum                             // +1 P=NamedIndId
    tell(triple(ShapeRegion,'http://knowrob.org/kb/urdf.owl#hasOrigin',Origin)), % connect ShapeRegion with Origin
    tell(triple(Origin, soma:hasPositionVector, term(Pos))), % set the position of Origin
    tell(triple(Origin, soma:hasOrientationVector, term(Rot))), % set the rotation of Origin
    tell(triple(ShapeRegion, soma:hasWidth, Width)), % set the depth of Shape
    tell(triple(ShapeRegion, soma:hasDepth, Depth)), % set the width of Shape
    tell(triple(ShapeRegion, soma:hasHeight, Height)), % set the height of Shape
    set_dimension_semantics(ObjID, Width, Depth, Height), % add aditional information for the shape like tall/small/flat...
    atom_number(ShapeConfAtom, ShapeConf), % save the Confidences in the db
    tell(triple(ObjID, hsr_objects:'ConfidenceShapeValue', ShapeConfAtom)),
    atom_number(ColorConfAtom, ColorConf),
    tell(triple(ObjID, hsr_objects:'ConfidenceColorValue', ColorConfAtom)),
    set_object_color(ObjID, Color, ColorConf),  % set the color
    tell(triple(ObjID, hsr_objects:'supportable', true)),     % identify the object as an supportable object this is used by suturo_existing_objects

    % todo: %%%%%%% remove me %%%%%%%%
    tell(triple(ObjID, hsr_objects:'supportedBy', table)),


    reachability_check([Width, Depth, Height], ObjID, Reachability), % check if object is reachable
    tell(triple(ObjID, hsr_objects:'hasReachability', Reachability)),

    %%% ================ visualization marker array publishpublish

    % TODO why not working with 1x ?
    marker_plugin:republish,
    marker_plugin:republish,
    !. % when call stack reaches here, then all bindings



%%% =========================== reachable predicates
reachability_check([Width, Depth, Height], ObjID, Reachability) :-
    % todo : refactor if supportedBySurface, then check if shelf: 30 cm or table 40 cm
    (
    check_too_big([Width, Depth, Height]) -> Reachability = 1;
    check_too_much_distance(ObjID) -> Reachability = 2;
    Reachability = 0
    ).

%% check_too_big([Width, Depth, Height]) is det.
%
%
%
%
check_too_big([Width, Depth, Height]) :-
    Width > 0.11;
    Depth > 0.11;
    Height > 0.11.
%% check_too_much_distance(ObjID) is det.
%
%
%
%
check_too_much_distance(ObjID) :-
    calc_distance(ObjID, Distance),
    Distance > 0.4.
%% calc_distance(ObjID, Distance) is det.
%
%
%
%
calc_distance(ObjID, Distance) :-
    % todo : refactor with existing functionalities
    % is_at('iai_kitchen/table_front_edge_center', [map,[AX,AY,AZ],_]),
    is_at('base_footprint', [map,[AX,AY,AZ],_]),
    is_at(ObjID, [map,[BX,BY,BZ],_]),
    DX is AX - BX,
    DY is AY - BY,
    DZ is AZ - BZ,
    Distance is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)).

%%% =========================== reachable reasons
% todo : think of a more clever mapping of reason representation <-> efficient querying
reachable_reason(0, Reachability) :-
    Reachability = 'Reachable'.
reachable_reason(1, Reachability) :-
    Reachability = 'Ungraspable because too big for gripper'.
reachable_reason(2, Reachability) :-
    Reachability = 'Object is out of reach'.
reachable_reason(3, Reachability) :-
    Reachability = 'Unreachable for unkown reason'.



set_test_for_graspable :-
    writeln('======= start tests'),
    % Set base footprint
    writeln('=== is at fooprint'),
    tell(is_at('base_footprint', [map, [0,0,0], [0,0,0,1]])),

    writeln('=== create objects'),
    %%%
    writeln('Reachable, 1st closest'),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#PringlesOriginals', 1,  ['map', [0.1,0.1,0.1], [0, 0, 0, 1]], [0.05, 0.05, 0.05], 'box',1, [0,0,255], 1, FirstObj),
    writeln(FirstObj),
    %%%
    writeln('Reachable, 2nd closest'),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#PringlesOriginals', 1,  ['map', [0.2,0.2,0.2], [0, 0, 0, 1]], [0.05, 0.05, 0.05], 'box',1, [0,0,255], 1, SecondObj),
    writeln(SecondObj),
    %%%
    writeln('Unreachable, too big'),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#PringlesOriginals', 1,  ['map', [0.1,0.1,0.1], [0, 0, 0, 1]], [0.5, 0.05, 0.05], 'box',1, [0,0,255], 1, ThirdObj),
    writeln(ThirdObj),
    %%%
    writeln('Unreachable, out of reach'),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#PringlesOriginals', 1,  ['map', [10,0.1,0.1], [0, 0, 0, 1]], [0.5, 0.05, 0.05], 'box',1, [0,0,255], 1, FourthObj),
    writeln(FourthObj),
    %%%

    writeln('=== all objects graspable'),
    all_objects_graspable(Graspable),
    length(Graspable, LenGraspable),
    LenGraspable =:= 2,
    writeln('=== ---> passed!'),

    writeln('=== all objects not graspable'),
    all_objects_not_graspable(NotGraspable),
    length(NotGraspable, LenNotGraspable),
    LenNotGraspable =:= 2,
    writeln('=== ---> passed!'),

    writeln('=== next graspable object on surface'),
    next_graspable_object_on_surface(NextGraspable, table),
    member(NextGraspable,[FirstObj, SecondObj]),
    writeln('=== ---> passed!'),

    writeln('=== all not graspable objects on surface'),
    all_not_graspable_objects_on_surfacte(NotGraspableObjects, table),
    length(NotGraspableObjects, LenNotGraspableObjects),
    LenNotGraspableObjects =:= 2,
    writeln('=== ---> passed!'),

    writeln('=== set_not_graspable'),
    set_not_graspable(FirstObj, table),
    all_not_graspable_objects_on_surfacte(MoreUngraspable, table),
    length(MoreUngraspable, LenMoreUngraspable),
    LenMoreUngraspable =:= 3,
    writeln('=== ---> passed!'),


    !.

%%% =============================== reachable objects for knowledge client
%% all_objects_graspable(Graspable) is nondet.
%
%
%
%
all_objects_graspable(Graspable):-
    findall(Subject,ask(triple(Subject, hsr_objects:'hasReachability',0)),Graspable).
%% all_objects_not_graspable(Graspable) is nondet.
%
%
%
%
all_objects_not_graspable(NotGraspable):-
    findall(Subject,ask(triple(Subject, hsr_objects:'hasReachability',>(0))),NotGraspable).
%% next_graspable_object_on_surface(NextGraspable, Surface) is nondet.
%3
%
%
%
next_graspable_object_on_surface(NextGraspable, Surface) :-
    % todo : sort for distance
    findall(Subject,ask(triple(Subject, hsr_objects:'supportedBy', Surface)),ObjectsOnSurface),
%    predsort(compareDistances, ObjectsOnSurface, SortedObjs),
    nth0(0, ObjectsOnSurface, NextGraspable).

%% all_not_graspable_objects_on_surfacte(Graspable, Surface) is nondet.
%
%
%
%
all_not_graspable_objects_on_surfacte(NotGraspable, Surface) :-
    findall(Subject,ask(triple(Subject, hsr_objects:'supportedBy', Surface)),ObjectsOnSurface).

%% set_not_graspable(Object, ReachabilityEnum) is det.
%
%
%
%
set_not_graspable(Object, ReachabilityEnum):-
    tell(triple(Object, hsr_objects:'hasReachability', ReachabilityEnum)).

%%% =========================== helper functions
%% calc_distance_from_surface(ObjID, Surface, Distance) is det.
%
%
%
%
calc_distance_from_surface(ObjID, Surface, Distance) :-
    % todo : refactor with existing functionalities
    is_at(Surface, [map,[AX,AY,AZ],_]),
    is_at(ObjID, [map,[BX,BY,BZ],_]),
    DX is AX - BX,
    DY is AY - BY,
    DZ is AZ - BZ,
    Distance is sqrt( ((DX*DX) + (DY*DY)) + (DZ*DZ)).
%% check_if_graspable(Object) is nondet.
%
%
%
%
check_if_graspable(Object) :-
    ask(triple(Object,hsr_objects:'hasReachability', 0)).



% Recursively create a Random String of a given length
random_id_gen(Size, Result):-
    ( Size > 0
    -> (Characters = ['A', 'a', 'B', 'b', 'C', 'c', 'D', 'd', 'E', 'e', 'F', 'f',
                      'G', 'g', 'H', 'h', 'I', 'i', 'J','j','K','k','L','l','M',
                      'm','N','n','O','o','P','p','Q','q','R','r','S','s','T',
                      't','U','u','V','v','W','w','X','x','Y','y','Z','z'],
        random(0, 48, RandomValue),
        nth0(RandomValue, Characters, RandomCharacter),
        random_id_gen(Size - 1, SubResult),
        atom_concat(RandomCharacter, SubResult, Result))
    ;   Result = '')
    .


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
object_type_handling(PerceivedObjectType, TypeConfidence, ObjectType) :-
    min_class_confidence(MinConf),
    (   number(TypeConfidence),
        TypeConfidence >= MinConf
        -> ObjectType = PerceivedObjectType;
        ObjectType = 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Other'
        ).


%%%%%%%%%% asserts Dimension Semantic is the object tall/flat/long/small/big? %%%%%%%%%%
% TODO is prob does not add multible informations
set_dimension_semantics(Instance, _, _, Height) :-
    Height > 0.16,
    tell(triple(Instance, hsr_objects:'size', 'tall')).

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Height < Width * 0.9,
    Height < Depth * 0.9,
    tell(triple(Instance, hsr_objects:'size', 'flat')).

set_dimension_semantics(Instance, Width, Depth, Height) :-
    ((Depth > Width * 2, Depth > Height * 2);
     (Width > Depth * 2, Width > Height * 2)),
    tell(triple(Instance, hsr_objects:'size', 'long')).

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Volume is Width * Depth * Height * 1000,
    Volume < 0.6,
    tell(triple(Instance, hsr_objects:'size', 'small')).

set_dimension_semantics(Instance, Width, Depth, Height) :-
    Volume is Width * Depth * Height * 1000,
    Volume > 2.0,
    tell(triple(Instance, hsr_objects:'size', 'big')).

% succeed even when no other semantic is set
set_dimension_semantics(_Instance,_W,_D,_H) :-
    true.




%%%%%%%%%% COLOR SEMANTICS %%%%%%%%%%

% When the Confidence is to low this querry will succeed and set an empty color
set_object_color(ObjID, _, Confidence) :-
    not(Confidence = 0), % for cases in which Perception does not give confidences.
    min_color_confidence(MinConf),
    Confidence < MinConf,
    tell(triple(ObjID, hsr_objects:'colour', '')),
    tell(object_color_rgb(ObjID, '')).

% Because the set_object_color(Instance, _, Confidence) gets executed before this is.
% We can just ignore the Confidence because we know it is high enough
set_object_color(ObjID, [R,G,B], _) :-    
    tell(has_type(ColorType, soma:'Color')),
    tell(triple(ObjID, soma:hasColor, ColorType)),
    tell(object_color_rgb(ObjID, [R,G,B])),
    triple(ColorType,dul:hasRegion,Region),
    tell(triple(Region, soma:hasTransparencyValue, 0)),
    RConv is R/255,    GConv is G/255,    BConv is B/255,
    set_color_semantics(ObjID, [RConv,GConv,BConv]).

set_color_semantics(ObjID, [0.0, 0.0, 0.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'dark')).

set_color_semantics(ObjID, [1.0, 0.0, 0.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'red')).

set_color_semantics(ObjID, [0.0, 1.0, 0.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'green')).

set_color_semantics(ObjID, [1.0, 1.0, 0.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'yellow')).

set_color_semantics(ObjID, [0.0, 0.0, 1.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'dark-blue')).

set_color_semantics(ObjID, [1.0, 0.0, 1.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'violet')).

set_color_semantics(ObjID, [0.0, 1.0, 1.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'light-blue')).

set_color_semantics(ObjID, [1.0, 1.0, 1.0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'bright')).

% Used so when no color is given the query does not fail
set_color_semantics(_, _) :-
    true.

