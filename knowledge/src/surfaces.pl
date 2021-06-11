
:- module(surfaces,
    [
    init_furnitures/0,
    is_furniture/1,
    all_furnitures/1,
    furniture_surfaces/2,
    surfaces_not_visited/1,
    bucket_surfaces/1,
    has_table_shape/1,
    has_shelf_shape/1,
    has_bucket_shape/1,
    has_surface/2,
    visited/1,
    update_visit_state/2,
    set_surface_visited/1,
    set_surface_not_visited/1,
    all_surfaces_of_type/2,
    supporting_surface/1,
    assert_object_on/2,
    %% FIND SURFACES
    all_surfaces/1, %replaces all_srdl_objects contains ground
    is_surface/1,
    % Get poses 
    pose_of_tables/1,
    pose_of_shelves/1,
    pose_of_buckets/1,
    pose_of_surfaces/2,
    compareDistances/3,
    %% FIND OBJs
    objects_on_surface/2,
    is_suturo_object/1,
    objects_on_furniture/2,
    objects_on_list_of_surfaces/2,
    all_objects_on_ground/1,
    all_objects_in_whole_shelf_/1, % will soon be deprecated
    all_objects_on_tables_/1,
    all_objects_in_buckets/1,    
    %% CREATE OBJECT
    place_object/1,
    get_perception_surface_region/2,
    %% TEMP
    create_furniture/2,
    assign_surfaces/3,
    init_visit_state/1
    ]).

:- tripledb_load(
	'http://knowrob.org/kb/URDF.owl',
	[ namespace(urdf, 'http://knowrob.org/kb/urdf.owl#')
	]).

:- rdf_meta % TODO FIX ME
    get_surface_id_by_name(r,?),
    supporting_surface(?),
    surface_big_enough(?),
    surface_big_enough(r,?),
    point_in_rectangle(r,r,r,r,r),
    assert_surface_types(?),
    object_goal_pose_offset(r,?,?),
    all_objects_in_whole_shelf(?),
    all_objects_on_source_surfaces(?),
    place_object(r),
    select_surface(r,?),
    object_goal_pose(r,?,?,?).



init_furnitures :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    findall(FurnitureLink, 
    (
        member(FurnitureLink, Links),
        (
            sub_string(FurnitureLink,_,_,_,"table_front_edge_center");
            sub_string(FurnitureLink,_,_,_,"shelf_base_center");
            sub_string(FurnitureLink,_,_,_,"bucket_front_edge_center")
        )
    ),
    FurnitureLinks),
    forall(member(FurnitureLink2, FurnitureLinks),
    (
        split_string(FurnitureLink2, ":", "", [_, Type, Shape]),
        create_furniture(Type, Furniture),
        tell(triple(Furniture, urdf:'hasURDFName', FurnitureLink2)),
        tell(has_type(FurnitureLocation, soma:'Location')),
        tell(triple(Furniture, dul:'hasLocation', FurnitureLocation)),
        assign_surfaces(Furniture, FurnitureLink2, Shape),
        init_visit_state(Furniture)
    )).


create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"armchair"),
    tell(has_type(Furniture, hsr_rooms:'Armchair')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bed"),
    tell(has_type(Furniture, hsr_rooms:'Bed')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bucket"),
    tell(has_type(Furniture, hsr_rooms:'Bucket')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"couch"),
    tell(has_type(Furniture, hsr_rooms:'Couch')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"cabinet"),
    tell(has_type(Furniture, hsr_rooms:'Cabinet')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"dishwasher"),
    tell(has_type(Furniture, hsr_rooms:'Dishwasher')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"fridge"),
    tell(has_type(Furniture, hsr_rooms:'Fridge')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"shelf"),
    tell(has_type(Furniture, hsr_rooms:'Shelf')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sideboard"),
    tell(has_type(Furniture, hsr_rooms:'Sideboard')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sidetable"),
    tell(has_type(Furniture, hsr_rooms:'Sidetable')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sink"),
    tell(has_type(Furniture, hsr_rooms:'Sink')),
    !.

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    tell(has_type(Furniture, hsr_rooms:'Table')),
    !.


assign_surfaces(Furniture, FurnitureLink, Shape) :-
    sub_string(Shape,_,_,_,"table"),
    tell(triple(Furniture, soma:'hasShape', hsr_rooms:'TableShape')),
    tell(has_type(FurnitureSurface, hsr_rooms:'Tabletop')),
    tell(triple(Furniture, hsr_rooms:'hasSurface', FurnitureSurface)),
    sub_atom(FurnitureLink, 0, _, 17, FurnitureStem),
    atom_concat(FurnitureStem, "center", SurfaceLink),
    tell(triple(FurnitureSurface, urdf:'hasURDFName', SurfaceLink)).


assign_surfaces(Furniture, FurnitureLink, Shape) :-
    sub_string(Shape,_,_,_,"shelf"),
    tell(triple(Furniture, soma:'hasShape', hsr_rooms:'ShelfShape')),
    get_urdf_id(URDF),
    urdf_link_child_joints(URDF, FurnitureLink, Joints),
    findall(SurfaceLink,
    (
        member(Joint, Joints),
        urdf_joint_child_link(URDF, Joint, SurfaceLink)
    ), 
    SurfaceLinks),
    forall((member(SurfaceLink, SurfaceLinks), supporting_surface(SurfaceLink)),
    (
        tell(has_type(FurnitureSurface, hsr_rooms:'Shelffloor')),
        tell(triple(Furniture, hsr_rooms:'hasSurface', FurnitureSurface)),
        tell(triple(FurnitureSurface, urdf:'hasURDFName', SurfaceLink))
    )).


assign_surfaces(Furniture, FurnitureLink, Shape) :-
    sub_string(Shape,_,_,_,"bucket"),
    tell(triple(Furniture, soma:'hasShape', hsr_rooms:'BucketShape')),
    tell(has_type(FurnitureSurface, hsr_rooms:'BucketOpening')),
    tell(triple(Furniture, hsr_rooms:'hasSurface', FurnitureSurface)),
    sub_atom(FurnitureLink, 0, _, 17, FurnitureStem),
    atom_concat(FurnitureStem, "surface_center", SurfaceLink),
    tell(triple(FurnitureSurface, urdf:'hasURDFName', SurfaceLink)).


init_visit_state(Furniture) :-
    furniture_surfaces(Furniture, Surfaces),
    forall(member(Surface, Surfaces),
    (
        tell(has_type(VisitState, hsr_rooms:'VisitState')),
        tell(triple(Surface, hsr_rooms:'hasVisitState', VisitState)),
        tell(triple(VisitState, hsr_rooms:'visited', true))
    )).


set_surface_visited(Surface) :-
    update_visit_state(Surface, true).

set_surface_not_visited(Surface) :-
    update_visit_state(Surface, false).

update_visit_state(Surface, State) :-
    is_surface(Surface),
    triple(Surface, hsr_rooms:'hasVisitState', VisitState),
    forall(triple(VisitState, hsr_rooms:'visited', _), tripledb_forget(VisitState, hsr_rooms:'visited', _)),
    tell(triple(VisitState, hsr_rooms:'visited', State)).

visited(Surface) :-
    is_surface(Surface),
    triple(Surface, hsr_rooms:'hasVisitState', VisitState),
    triple(VisitState, hsr_rooms:'visited', Visited),
    not Visited == 0.

surfaces_not_visited(Surfaces) :-
    findall(Surface,
    (
        is_surface(Surface),
        not visited(Surface)
    ),
    Surfaces).


has_table_shape(Surface) :-
    has_surface(Furniture, Surface),
    triple(Furniture, soma:'hasShape', hsr_rooms:'TableShape').

has_shelf_shape(Surface) :-
    has_surface(Furniture, Surface),
    triple(Furniture, soma:'hasShape', hsr_rooms:'ShelfShape').

has_bucket_shape(Surface) :-
    has_surface(Furniture,Surface),
    triple(Furniture, soma:'hasShape', hsr_rooms:'BucketShape').


is_furniture(Furniture) :-
    has_type(Furniture, soma:'DesignedFurniture').

all_furnitures(Furnitures) :-
    findall(Furniture,
    (
        has_type(Furniture, soma:'DesignedFurniture')
    ),
    Furnitures).

all_furnitures_of_type(FurnitureType, Furnitures) :-
    findall(Furniture, has_type(Furniture, FurnitureType), Furnitures).

is_surface(Surface) :-
    has_type(Surface, soma:'Surface').

all_surfaces(Surfaces) :-
    findall(Surface, 
    (
        has_type(Surface, soma:'Surface')
    ),
    Surfaces).


bucket_surfaces(BucketSurfaces) :-
    findall(BucketSurface,
    (
        has_type(Furniture, hsr_rooms:'Bucket'),
        has_surface(Furniture, BucketSurface)
    ),
    BucketSurfaces).


has_surface(Furniture, Surface) :-
    triple(Furniture, hsr_rooms:'hasSurface', Surface).


furniture_surfaces(Furniture, Surfaces) :-
    findall(Surface,
    (
        has_surface(Furniture, Surface)
    ),
    Surfaces).


%% supporting_surface(?Surface).
%
supporting_surface(SurfaceLink):-
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF,SurfaceLink,ShapeTerm,_),
    surface_big_enough(ShapeTerm).

surface_big_enough(box(X, Y, _)):- %TODO Support other shapes, has not been tested yet.
    square_big_enough(X,Y).

%surface_big_enough(mesh(_,[X,Y,_])):- % TODO? Support Meshes (They are all asserted with size 1, 1, 1)
%    square_big_enough(X,Y).

square_big_enough(X,Y):- %TODO Support other shapes
    Size = X * Y,
    (  Size >= 0.09 , X > 0.2, Y > 0.2
    -> true
    ; fail
    ).

% needs to be put in beliefstate
assert_object_on(ObjectInstance, SurfaceLink) :-
    all_surfaces(SurfaceLinks), % this makes sure, we actually have a surface here
    member(SurfaceLink,SurfaceLinks),
    has_location(ObjectLocation, ObjectInstance),
    tripledb_forget(ObjectInstance, hsr_objects:'isSupportedBy', _),
    tripledb_tell(ObjectInstance, hsr_objects:'isSupportedBy', SurfaceLink).


/**
*****************************************FIND SURFACES******************************************************
*/

all_surfaces_of_type(SurfaceType, Surfaces) :-
    findall(Surface, 
    (
        has_type(Type, SurfaceType),
        triple(Surface, hsr_objects:'isSurfaceType', Type)
    ),
    Surfaces).


is_suturo_object(Object) :-
    hsr_existing_objects(Objects),
    member(Object, Objects).


%%%%%%%%%%% Get Poses %%%%%%%%%%%%%%%%%%%%%%%%%%

pose_of_tables(Positions) :-
    table_surfaces(Tables),
    pose_of_surfaces(Tables, Positions).

pose_of_shelves(Positions):-
    shelf_surfaces(Shelves),
    pose_of_surfaces(Shelves, Positions).

pose_of_buckets(Positions) :-
    bucket_surfaces(Buckets),
    pose_of_surfaces(Buckets, Positions).

pose_of_source_surfaces(Positions) :-
    all_source_surfaces(Surfaces),
    pose_of_surfaces(Surfaces, Positions).

% Sorts the surfaces by Distance to the robot and returns a List of their positions in that order.
pose_of_surfaces(Surfaces, Positions) :-
    predsort(compareDistances, Surfaces, SortedSurfaces),
    maplist(surface_pose_in_map, SortedSurfaces, Positions).

% compares the Distance of two things (Surface or Object) to the Robot based on compare/3.
compareDistances(Order, Thing1, Thing2) :-
    distance_to_robot(Thing1, Dist1),
    distance_to_robot(Thing2, Dist2),
    (Dist1 = Dist2 % prevent predsort from deleting duplicate distances
        -> compare(Order, 0, Dist2)
        ; compare(Order, Dist1, Dist2))
    .


/**
*****************************************FIND OBJECTS******************************************************
*/

objects_on_surface(ObjectInstances, Surface) :-
    % place_objects,
    findall(ObjectInstance,
        (
        triple(ObjectLocation, soma:'isSupportedBy', Surface),
        once(has_location(ObjectInstance,ObjectLocation))
        ),
        ObjectInstances).


% Objs is a list of all Objects on all source surfaces.
all_objects_on_source_surfaces(Objs):-
    all_source_surfaces(Surfaces),
    objects_on_list_of_surfaces(Objs, Surfaces).

objects_on_list_of_surfaces(ObjectInstances, SurfaceList):-
    findall(Objects,
    ( 
        member(Surface, SurfaceList),
        objects_on_surface(Objects, Surface)
    ),
        ObjectsNested),
    flatten(ObjectsNested, ObjectInstances).

objects_on_furniture(Furniture_ID, Objects):-
    furniture_surfaces(Furniture_ID, Surfaces),
    objects_on_list_of_surfaces(Objects,Surfaces).

all_objects_on_ground(Instances) :-
    findall(Instance, (
        ground_surface(Ground),
        objects_on_surface(ObjOnGround, Ground),
        member(Instance, ObjOnGround)
        ), Instances).


all_objects_in_whole_shelf_(Instances) :-
    findall(Instance, (
        shelf_surfaces(ShelveLinks),
        member(Shelf, ShelveLinks),
        objects_on_surface(ObjPerShelf, Shelf),
        member(Instance, ObjPerShelf)
        ), Instances).


all_objects_on_tables_(Instances) :-
    findall(Instance, (
        table_surfaces(TableLinks),
        member(Table, TableLinks),
        objects_on_surface(ObjPerTable, Table),
        member(Instance, ObjPerTable)
        ), Instances).

all_objects_in_buckets(Instances) :-
    findall(Instance, (
        bucket_surfaces(Buckets),
        member(Bucket, Buckets),
        objects_on_surface(ObjPerBucket, Bucket),
        member(Instance, ObjPerBucket)
        ), Instances).    

 % DEPRECATED! Use only for backward compatibility reasons
all_objects_on_table(Instances) :- 
    all_objects_on_tables_(Instances).


all_objects_in_gripper(Instances):-
    findall(Instance, (
        objects_on_surface(Objs, gripper),
        member(Instance, Objs)
        ), Instances).



/**
***************************************************ROLES*********************************************
*/

% Gives a list of surfaces the role source
make_surfaces_source(Surfaces):-
    forall(member(Surface, Surfaces), make_role(Surface, source)).

% Gives a list of surfaces the role target
make_surfaces_target(Surfaces):-
    forall(member(Surface, Surfaces), make_role(Surface, target)).


% Gives the gives SurfaceLink the Role (target or source)
make_role(SurfaceLink, Role):-
    forall(triple(SurfaceLink,hsr_objects:'sourceOrTarget',R), tripledb_forget(SurfaceLink,hsr_objects:'sourceOrTarget',R)),
    %rdf_retractall(SurfaceLink, hsr_objects:'sourceOrTarget',_),
    tell(triple(SurfaceLink, hsr_objects:'sourceOrTarget', Role)).


% Role is the role (target or source) of the given SurfaceLink
get_surface_role(SurfaceLink, Role):-
    triple(SurfaceLink, hsr_objects:'sourceOrTarget', Role).

get_perception_surface_region(Surface, PerceptionName):-
    has_shelf_shape(Surface),
    has_urdf_name(Surface,Name),
    split_string(Surface, ":","",SurfaceSplit),
    nth0(0,SurfaceSplit,Name),sub_atom(Surface, _, 1, 0, Number),
    string_concat(Name,"_floor_",Temp),
    string_concat(Temp,Number,PerceptionName),!.

get_perception_surface_region(Surface, PerceptionName):-
    not(has_shelf_shape(Surface)),    
    has_urdf_name(Surface,Name),
    split_string(Name, ":","",SurfaceSplit),
    nth0(0,SurfaceSplit,PerceptionName).

