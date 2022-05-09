:- module(object_creation, 
    [
        create_object/9
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(soma, 'https://ease-crc.github.io/soma/owl/current/SOMA.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://www.knowrob.org/kb/knowrob.owl#', [keep(true)]).


:- use_module(library('ros/marker/marker_plugin'), [marker_message_new/3, republish/0]). % Importing marker_plugin
:- use_module(library('model/objects/object_validation'), 
    [
        object_size_ok/1, 
        validate_confidence/3,
        object_type_handling/3
    ]).

% TODO rdf meta
:- rdf_meta
    create_object(r,?),
	object_at(r,r,r,?),
	object_at_table(?),
	object_of_type(r,?),
	create_object_at(r,r,r,?,-,-),
	hsr_existing_objects(?).


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
% TODO refactor into more suitable predicates
% TODO change that terriple param order
% TODO improve 'box' param
% TODO fix the marker_plugin Warnings
% TODO go over all db writings, where to we actually need a tell ?
% TODO validate reachable in create_object: @param for colision avoidance, inferr distance via self position + obj position, inferr size of gripper and check with the existing size
create_object(PerceivedObjectType, PercTypeConf, [Frame,Position,Rotation], [Width, Depth, Height], 'box', PercShapeConf, Color, PercColorConf, ObjID):-

    format(string(Logmsg),"create_object(~n  Type: ~w,~n  TypeConf: ~w,~n  FPR: ~w,~n  WDH: ~w,~n  'box',~n  ShapeConf: ~w,~n  Color: ~w,~n  ColorConf:~w,~n  ObjID~n)",[
	       PerceivedObjectType,
	       PercTypeConf,
	       [Frame,Position,Rotation],
	       [Width, Depth, Height],
	       PercShapeConf,
	       Color,
	       PercColorConf
	   ]),
    ros_info(Logmsg),
    
    %%% ================ Object validation
    % TODO make this dynamic to constraints
    (
	object_size_ok([Width, Depth, Height]) -> true; % Dont add the object when the size is to big/small
	(ros_info("Object size not ok"), fail())
    ),
    (
	validate_confidence(class, PercTypeConf, TypeConf) -> true;
	(ros_info("Class confidence not ok"), fail())
    ),
    (
	validate_confidence(shape, PercShapeConf, ShapeConf) -> true;
	(ros_info("Shape confidence not ok"), fail())
    ),
    (
	validate_confidence(color, PercColorConf, ColorConf) -> true;
	(ros_info("Color confidence not ok"), fail())
    ),
    object_type_handling(PerceivedObjectType, PercTypeConf, ObjectType), % When the PercTypeConf is to low the Type is set to Other, Otherwise ObjectType is the same as PerceivedObjectType

    %%% ================ Object creation
    tell(has_type(ObjID, ObjectType)), % Create Object of type ObjectType           // +1 S=ObjID
    tell(is_physical_object(ObjID)), % write into triple: ont: is-a                 // +1 P=ObjID
    tell(is_at(ObjID,[Frame,Position,Rotation])), % this triggers rosprolog to publish it on tf     // no change! no tell(...) needed? doch needed..
    atom_number(TypeConfidenceAtom, TypeConf),
    tell(triple(ObjID, hsr_objects:'hasConfidenceClassValue', TypeConfidenceAtom)), %  // +1 P=ObjID
    ((triple(ObjID, soma:hasShape, Shape), % check if Shape exists                  // +6, 1x P=ObjID, 3x P=ShapeID, 2x P=ShapeRegionID
    triple(Shape,dul:hasRegion,ShapeRegion)); % if yes, then check if ShapeRegion exist
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
    tell(triple(ObjID, hsr_objects:'hasConfidenceShapeValue', ShapeConfAtom)),
    atom_number(ColorConfAtom, ColorConf),
    tell(triple(ObjID, hsr_objects:'hasConfidenceColorValue', ColorConfAtom)),
    set_object_color(ObjID, Color, ColorConf),  % set the color
    tell(triple(ObjID, hsr_objects:'supportable', true)),     % identify the object as an supportable object this is used by suturo_existing_objects

    tell(has_type(HandleState, hsr_objects:'HandleState')),
    tell(triple(ObjID, hsr_objects:'hasHandleState', HandleState)),
    tell(triple(HandleState, hsr_objects:'handeled', false)),

    tell(has_type(ObjectLocation, soma:'Location')),
    tell(triple(ObjID, dul:'hasLocation', ObjectLocation)),

    % Used by clean_table to get the pose where the objects should be brought back to
    (store_starting_location(ObjID, Frame,Position,Rotation) -> true;ros_info("store_starting_location failed")),

    %%% ================ visualization marker array publish
    % TODO why not working with 1x ?
    marker_plugin:republish,
    marker_plugin:republish,
    ros_info("create_object finished").


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

set_color_semantics(ObjID, [0, 0, 0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'dark')).

set_color_semantics(ObjID, [1, 0, 0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'red')).

set_color_semantics(ObjID, [0, 1, 0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'green')).

set_color_semantics(ObjID, [1, 1, 0]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'yellow')).

set_color_semantics(ObjID, [0, 0, 1]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'dark-blue')).

set_color_semantics(ObjID, [1, 0, 1]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'violet')).

set_color_semantics(ObjID, [0, 1, 1]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'light-blue')).

set_color_semantics(ObjID, [1, 1, 1]) :-
    tell(triple(ObjID, hsr_objects:'colour', 'bright')).

% Used so when no color is given the query does not fail
set_color_semantics(_, _) :-
    true.

store_starting_location(ObjID, Frame, [X, Y, Z], [RX, RY, RZ, RW]) :-
    tell(triple(ObjID, suturo:'start_pose_frame', Frame)),
    
    tell(triple(ObjID, suturo:'start_pose_x', X)),
    tell(triple(ObjID, suturo:'start_pose_y', Y)),
    tell(triple(ObjID, suturo:'start_pose_z', Z)),

    tell(triple(ObjID, suturo:'start_pose_rx', RX)),
    tell(triple(ObjID, suturo:'start_pose_ry', RY)),
    tell(triple(ObjID, suturo:'start_pose_rz', RZ)),
    tell(triple(ObjID, suturo:'start_pose_rw', RW)).
