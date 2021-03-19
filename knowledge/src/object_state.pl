% TODO rdf meta
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
    set_color_semantics/2    
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).


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

% forget a specific object
hsr_forget_object(Object) :-
    forall(triple(Object,X,Y), tripledb_forget(Object,X,Y)).
    % TODO we need to stop publishing the tf and marker


forget_objects_on_surface_(SurfaceLink) :-
    objects_on_surface(Objs,SurfaceLink),
    member(Obj,Objs),
    hsr_forget_object(Obj).


%%
% finds the surface an object was seen on. When there is no surface supporting the object and
% the center point of the object < 0.5 the object is placed on the ground. Otherwise the query resolves to false.
place_object(Object):-
    object_supportable_by_surface(Object, Surface),
    assert_object_on(Object,Surface).

% Transform should be [RelativeTFFrame,[X,Y,Z],[X,Y,Z,W]]
% Conf between 0 and 1
% Color needs to be [R,G,B] Values between 0 and 255
create_object(PerceivedObjectType, PercTypeConf, Transform, [Width, Depth, Height], Shape, PercShapeConf, Color, PercColorConf, ObjID):-
    % Dont add the object when the size is to big/small
    object_size_ok([Width, Depth, Height]),
    validate_confidence(class, PercTypeConf, TypeConf),
    validate_confidence(shape, PercShapeConf, ShapeConf),
    validate_confidence(color, PercColorConf, ColorConf),
    % When the PercTypeConf is to low the Type is set to Other, Otherwise ObjectType is the same as PerceivedObjectType
    object_type_handling(PerceivedObjectType, PercTypeConf, ObjectType),
    % create ID = Type + random id
    random_id_gen(6, Result),
    atom_concat(ObjectType, '_', ObjectTypeU),
    atom_concat(ObjectTypeU, Result, GenID),
    % TODO check if the ID is already used
    % Create Object of type ObjectType
    tell(has_type(GenID, ObjectType)),
    tell(is_physical_object(GenID)),
    tell(is_individual(Shape)),
    tell(triple(GenID, soma:hasShape, Shape)),
    % Save the transform
    tell(is_at(GenID,Transform)),
    % Save the Type Confidence
    atom_number(TypeConfidenceAtom, TypeConf),
    tell(triple(GenID, hsr_objects:'ConfidenceClassValue', TypeConfidenceAtom)),
    % Save object dimensions
    tell(triple(GenID, soma:hasShape, Shape)),
    tell(object_dimensions(GenID, Depth, Width, Height)),
    % add aditional information for the shape like tall/small/flat...
    set_dimension_semantics(GenID, Width, Depth, Height),   
    % save the Confidences in the db
    atom_number(ShapeConfAtom, ShapeConf),
    tell(triple(GenID, hsr_objects:'ConfidenceShapeValue', ShapeConfAtom)),
    atom_number(ColorConfAtom, ColorConf),
    tell(triple(GenID, hsr_objects:'ConfidenceColorValue', ColorConfAtom)),
    % set the color
    set_object_color(GenID, Color, ColorConf),
    % identify the object as an supportable object this is used by suturo_existing_objects
    tell(triple(GenID, hsr_objects:'supportable', true)),
    
    % create Marker for the Object
    % TODO create the Marker

    !.







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
        -> ObjectType = PerceivedObjectType
        ; ObjectType = 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Other'
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
    tell(is_individual(Color)),
    tell(triple(ObjID, soma:hasColor, Color)),
    tell(object_color_rgb(ObjID, [R,G,B])),
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

