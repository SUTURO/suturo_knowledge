
:- module(surfaces,
    [
    init_furnitures/0,
    is_furniture/1,
    all_furnitures/1,
    furniture_surfaces/2,
    furnitures_not_visited/1,
    has_table_shape/1,
    has_shelf_shape/1,
    has_bucket_shape/1,
    has_surface/2,
    visited/1,
    assign_visit_state/2,
    supporting_surface/1,
    assert_object_on/2,
    surface_type_of/2,
    %% FIND SURFACES
    all_surfaces/1, %replaces all_srdl_objects contains ground
    is_surface/1,
    is_table/1,
    is_bucket/1,
    is_shelf/1,
    is_bed/1,
    is_cabinet/1,
    is_couch/1,
    is_dishwasher/1,
    is_fridge/1,
    is_sideboard/1,
    is_sink/1,
    all_source_surfaces/1,
    all_target_surfaces/1,
    ground_surface/1,
    shelf_surfaces/1, 
    big_shelf_surfaces/1, % will soon be deprecated
    shelf_floor_at_height/2, % will soon be deprecated
    table_surfaces/1, 
    bucket_surfaces/1,
    all_surfaces_of_type/2,
    is_legal_obj_position/1,
    find_supporting_surface/2,
    % Get poses 
    pose_of_tables/1,
    pose_of_shelves/1,
    pose_of_buckets/1,
    pose_of_target_surfaces/1,
    pose_of_source_surfaces/1,
    pose_of_surfaces/2,
    compareDistances/3,
    %% FIND OBJs
    objects_on_surface/2,
    is_suturo_object/1,
    objects_on_list_of_surfaces/2,
    all_objects_on_source_surfaces/1,
    all_objects_on_target_surfaces/1,
    all_objects_on_ground/1,
    all_objects_in_whole_shelf_/1, % will soon be deprecated
    all_objects_on_tables_/1,
    all_objects_in_buckets/1,
    all_objects_on_table/1, % DEPRECATED! Use only for backward compatibility reasons
    %% CREATE OBJECT
    place_object/1,
    %% ROLES
    make_all_surface_type_role/2,
    make_surfaces_source/1,
    make_surfaces_target/1,
    make_role/2,
    get_surface_role/2
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
    object_goal_surface(r,?),
    object_goal_surface(r,?,?),
    object_goal_surface(r,?,?,?),
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
        split_string(FurnitureLink2, "#", "", [_, Type, Shape]),
        create_furniture(Type, Furniture),
        tell(triple(Furniture, urdf:'hasURDFName', FurnitureLink2)),
        init_visit_state(Furniture),
        assign_surfaces(Furniture, FurnitureLink2, Shape)
    )).


create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"armchair"),
    tell(has_type(Furniture, hsr_rooms:'Armchair')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bed"),
    tell(has_type(Furniture, hsr_rooms:'Bed')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"bucket"),
    tell(has_type(Furniture, hsr_rooms:'Bucket')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"couch"),
    tell(has_type(Furniture, hsr_rooms:'Couch')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"cabinet"),
    tell(has_type(Furniture, hsr_rooms:'Cabinet')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"dishwasher"),
    tell(has_type(Furniture, hsr_rooms:'Dishwasher')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"fridge"),
    tell(has_type(Furniture, hsr_rooms:'Fridge')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"shelf"),
    tell(has_type(Furniture, hsr_rooms:'Shelf')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sideboard"),
    tell(has_type(Furniture, hsr_rooms:'Sideboard')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sidetable"),
    tell(has_type(Furniture, hsr_rooms:'Sidetable')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"sink"),
    tell(has_type(Furniture, hsr_rooms:'Sink')).

create_furniture(FurnitureType, Furniture) :-
    sub_string(FurnitureType,_,_,_,"table"),
    tell(has_type(Furniture, hsr_rooms:'Table')).


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
        tell(triple(FurnitureSurface, urdf:'hasURDFName', FurnitureLink))
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
    tell(has_type(VisitState, hsr_rooms:'VisitState')),
    tell(triple(Furniture, hsr_rooms:'hasVisitState', VisitState)),
    tell(triple(VisitState, hsr_rooms:'visited', false)).


assign_visit_state(Furniture, State) :-
    triple(Furniture, hsr_rooms:'hasVisitState', VisitState),
    forall(triple(VisitState, hsr_rooms:'visited', _), tripledb_forget(VisitState, hsr_rooms.'visited', _)),
    tell(triple(VisitState, hsr_rooms:'visited', State)).

visited(Furniture) :-
    triple(Furniture, hsr_rooms:'hasVisitState', VisitState),
    triple(VisitState, hsr_rooms:'visited', Visited),
    (Visited == false
    -> fail).

furnitures_not_visited(Furnitures) :-
    findall(Furniture,
    (
        is_furniture(Furniture),
        not visited(Funiture)
    ),
    Furnitures).


has_table_shape(Furniture) :-
    triple(Furniture, soma:'hasShape', hsr_rooms:'TableShape').

has_shelf_shape(Furniture) :-
    triple(Furniture, soma:'hasShape', hsr_rooms:'ShelfShape').

has_bucket_shape(Furniture) :-
    triple(Furniture, soma:'hasShape', hsr_rooms:'BucketShape').


is_furniture(Furniture) :-
    has_type(Furniture, soma:'DesignedFurniture').

all_furnitures(Furnitures) :-
    findall(Furniture,
    (
        has_type(Furniture, soma:'DesignedFurniture')
    ),
    Furnitures).

is_surface(Surface) :-
    has_type(Surface, soma:'Surface').

all_surfaces(Surfaces) :-
    findall(Surface, 
    (
        has_type(Surface, soma:'Surface')
    ),
    Surfaces).


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
    tripledb_forget(ObjectInstance, hsr_objects:'supportedBy', _),
    tripledb_tell(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).


surface_type_of(Surface, Type):- % has not been tested yet.
    triple(Surface, hsr_objects:'isSurfaceType', Type).


is_table(Table) :-
    table_surfaces(Tables),
    member(Table, Tables).

is_shelf(Shelf) :-
    shelf_surfaces(Shelves),
    member(Shelf, Shelves).

is_bucket(Bucket) :-
    bucket_surfaces(Buckets),
    member(Bucket, Buckets).

is_bed(Bed) :-
    bed_surfaces(Beds),
    member(Bed, Beds).

is_cabinet(Cabinet) :-
    cabinet_surfaces(Cabinets),
    member(Cabinet, Cabinets).

is_couch(Couch) :-
    couch_surfaces(Couches),
    member(Couch, Couches).

is_dishwasher(Dishwasher) :-
    dishwasher_surfaces(Dishwashers),
    member(Dishwasher, Dishwashers).

is_fridge(Fridge) :-
    fridge_surfaces(Fridges),
    member(Fridge, Fridges).

is_sideboard(Sideboard) :-
    sideboard_surfaces(Sideboards),
    member(Sideboard, Sideboards).

is_sink(Sink) :-
    sink_surfaces(Sinks),
    member(Sink, Sinks).


/**
*****************************************FIND SURFACES******************************************************
*/

all_surfaces(SurfaceLinks):-
    findall(SurfaceLink,
        triple(SurfaceLink,hsr_objects:'isSurfaceType',_),
        SurfaceLinks
    ).


% Surfaces is a list of all SurfaceLinks that are source
all_source_surfaces(Surfaces):-
    all_surfaces(ExistingSurfaces),
    findall(Surface,
    (
        member(Surface, ExistingSurfaces),
        triple(Surface, hsr_objects:'sourceOrTarget', source)
    ),
        Surfaces).


% Surfaces is a list of all SurfaceLinks that are target
all_target_surfaces(Surfaces):-
    all_surfaces(ExistingSurfaces),
    findall(Surface,
    (
        member(Surface, ExistingSurfaces),
        triple(Surface, hsr_objects:'sourceOrTarget', target)
    ),
        Surfaces).


all_surfaces_of_type(SurfaceType, Surfaces) :-
    findall(Surface, 
    (
        has_type(Type, SurfaceType),
        triple(Surface, hsr_objects:'isSurfaceType', Type)
    ),
    Surfaces).


ground_surface(GroundSurface):-
    GroundSurface = ground.


shelf_surfaces(ShelfLinks):-
    findall(ShelfLink, 
    (
        has_type(Shelf, hsr_objects:'Shelf'),
        triple(ShelfLink, hsr_objects:'isSurfaceType',Shelf)
    ),
    ShelfLinks).


big_shelf_surfaces(ShelfLinks) :- % has not been tested yet.
    findall(ShelfLink,
    (
        triple(ShelfLink, hsr_objects:'isSurfaceType',shelf),
        not(sub_string(ShelfLink,_,_,_,small))
    ),
    ShelfLinks).


% Deprecated
shelf_floor_at_height(Height, TargetShelfLink) :- % has not been tested yet.
    findall(ShelfFloorLink, (
        big_shelf_surfaces(AllFloorsLinks),
        member(ShelfFloorLink, AllFloorsLinks),
        surface_pose_in_map(ShelfFloorLink, [[_,_,Z],_]),
        Z < Height
    ), ShelfFloorsLinks),
    reverse(ShelfFloorsLinks, [TargetShelfLink|_]).


table_surfaces(TableLinks):-
    findall(TableLink, 
        (
            has_type(Table, hsr_objects:'Table'),
            triple(TableLink, hsr_objects:'isSurfaceType',Table)
        ), 
    TableLinks).


bucket_surfaces(BucketLinks):-
    findall(BucketLink, 
    (
        has_type(Bucket, hsr_objects:'Bucket'),
        triple(BucketLink, hsr_objects:'isSurfaceType',Bucket)
    ), 
    BucketLinks).


bed_surfaces(BedLinks) :-
    findall(BedLink,
    (
        has_type(Bed, hsr_objects:'Bed'),
        triple(BedLink, hsr_objects:'isSurfaceType', Bed)
    ),
    BedLinks).


cabinet_surfaces(CabinetLinks) :-
    findall(CabinetLink,
    (
        has_type(Cabinet, hsr_objects:'Cabinet'),
        triple(CabinetLink, hsr_objects:'isSurfaceType', Cabinet)
    ),
    CabinetLinks).


couch_surfaces(CouchLinks) :-
    findall(CouchLink,
    (
        has_type(Couch, hsr_objects:'Couch'),
        triple(CouchLink, hsr_objects:'isSurfaceType', Couch)
    ),
    CouchLinks).

dishwasher_surfaces(DishwasherLinks) :-
    findall(DishwasherLink,
    (
        has_type(Dishwasher, hsr_objects:'Dishwasher'),
        triple(DishwasherLink, hsr_objects:'isSurfaceType', Dishwasher)
    ),
    DishwasherLinks).

fridge_surfaces(FridgeLinks) :-
    findall(FridgeLink,
    (
        has_type(Fridge, hsr_objects:'Fridge'),
        triple(FridgeLink, hsr_objects:'isSurfacRype', Fridge)
    ),
    FridgeLinks).

sideboard_surfaces(SideboardLinks) :-
    findall(SidebaordLink,
    (
        has_type(Sideboard, hsr_objects:'Sideboard'),
        triple(SidebaordLink, hsr_objects:'isSurfaceType', Sideboard)
    ),
    SideboardLinks).

sink_surfaces(SinkLinks) :-
    findall(SinkLink,
    (
        has_type(Sink, hsr_objects:'Sink'),
        triple(SinkLink, hsr_objects:'isSurfaceType', Sink)
    ),
    SinkLinks).

find_supporting_surface(Object, Surface) :-
    triple(Object, hsr_objects:'supportedBy', Surface).

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

pose_of_target_surfaces(Positions) :-
    all_target_surfaces(Surfaces),
    pose_of_surfaces(Surfaces, Positions).

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

objects_on_surface(ObjectInstances, SurfaceLink) :-
    findall(ObjectInstance,
        find_supporting_surface(ObjectInstance, SurfaceLink),
        ObjectInstances).


% Objs is a list of all Objects on all source surfaces.
all_objects_on_source_surfaces(Objs):-
    all_source_surfaces(Surfaces),
    objects_on_list_of_surfaces(Objs, Surfaces).


% Objs is a list of all Objects on all target surfaces.
all_objects_on_target_surfaces(Objs):-
    all_target_surfaces(Surfaces),
    objects_on_list_of_surfaces(Objs, Surfaces).



objects_on_list_of_surfaces(ObjectInstances, SurfaceList):-
    findall(Obj,
    ( 
        member(Surface, SurfaceList),
        objects_on_surface(Objects, Surface),
        member(Obj, Objects)
    ),
        ObjectInstances).


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


% Gives all surfaces with given name (ground, table, basket or shelf) the Role (target or source)
make_all_surface_type_role(SurfaceType, Role):-
    SurfaceType = ground,
    make_role(SurfaceType, Role).

make_all_surface_type_role(SurfaceType, Role):-
    forall(triple(SurfaceLink, hsr_objects:'isSurfaceType',SurfaceType), make_role(SurfaceLink,Role)).


% Gives the gives SurfaceLink the Role (target or source)
make_role(SurfaceLink, Role):-
    forall(triple(SurfaceLink,hsr_objects:'sourceOrTarget',R), tripledb_forget(SurfaceLink,hsr_objects:'sourceOrTarget',R)),
    %rdf_retractall(SurfaceLink, hsr_objects:'sourceOrTarget',_),
    tell(triple(SurfaceLink, hsr_objects:'sourceOrTarget', Role)).


% Role is the role (target or source) of the given SurfaceLink
get_surface_role(SurfaceLink, Role):-
    triple(SurfaceLink, hsr_objects:'sourceOrTarget', Role).

