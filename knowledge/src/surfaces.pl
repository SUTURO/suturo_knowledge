
:- module(surfaces,
    [
    assert_surface_types/1,
    init_surface_types/0,
    supporting_surface/1,
    assert_object_on/2,
    surface_type_of/2,
    is_legal_obj_position/1,
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
    is_object/1,
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


% Surface Link is the String used directly like /table_1_center etc.
assert_surface_types(SurfaceLink):-
    tell(triple(ground,hsr_objects:'isSurfaceType',ground)),
    supporting_surface(SurfaceLink), % Checks if the Collision is big enough to be a surface
    ( sub_string(SurfaceLink,_,_,_,shelf) % when the Link has the string shelf in it it is a shelf
    ->(has_type(Shelf, hsr_objects:'Shelf'), tell(triple(SurfaceLink,hsr_objects:'isSurfaceType',Shelf)))
    ;
    ( sub_string(SurfaceLink,_,_,_,table) % when the Link has the string table in it it is a table
    ->(has_type(Table, hsr_objects:'Table'), tell(triple(SurfaceLink,hsr_objects:'isSurfaceType',Table)))
    ;
    ( sub_string(SurfaceLink,_,_,_,bucket) % when the Link has the string bucket in it it is a bucket
    ->(has_type(Bucket, hsr_objects:'Bucket'), tell(triple(SurfaceLink,hsr_objects:'isSurfaceType',Bucket)))
    ;
    ( sub_string(SurfaceLink,_,_,_,cabinet) % when the link has the string cabinet in it it is a cabinet
    ->(has_type(Cabinet, hsr_objects:'Cabinet'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Cabinet)))
    ;
    ( sub_string(SurfaceLink,_,_,_,couch) % when the link has the string couch in it it is a couch
    ->(has_type(Couch, hsr_objects:'Couch'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Couch)))
    ;
    ( sub_string(SurfaceLink,_,_,_,dishwasher) % when the link has the string dishwasher in it it is a dishwasher
    ->(has_type(Dishwasher, hsr_objects:'Dishwasher'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Dishwasher)))
    ;
    ( sub_string(SurfaceLink,_,_,_,fridge) % when the link has the string fridge in it it is a fridge
    ->(has_type(Fridge, hsr_objects:'Fridge'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Fridge)))
    ;
    ( sub_string(SurfaceLink,_,_,_,sideboard) % when the link has the string sideboard in it it is a sideboard
    ->(has_type(Sideboard, hsr_objects:'Sideboard'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Sideboard)))
    ;
    ( sub_string(SurfaceLink,_,_,_,sink) % when the link has the string sink in it it is a sink
    ->(has_type(Sink, hsr_objects:'Sink'), tell(triple(SurfaceLink, hsr_objects:'isSurfaceType', Sink)))
    ;
    tell(triple(SurfaceLink,hsr_objects:'isSurfaceType',other))) % when it is not a shelf/table/bucket
    )))))))).


init_surface_types :-
    tell(has_type(Bed, hsr_objects:'Bed')),
    tell(has_type(Bucket, hsr_objects:'Bucket')),
    tell(has_type(Cabinet, hsr_objects:'Cabinet')),
    tell(has_type(Couch, hsr_objects:'Couch')),
    tell(has_type(Dishwasher, hsr_objects:'Dishwasher')),
    tell(has_type(Fridge, hsr_objects:'Fridge')),
    tell(has_type(Shelf, hsr_objects:'Shelf')),
    tell(has_type(Sideboard, hsr_objects:'Sideboard')),
    tell(has_type(Sink, hsr_objects:'Sink')),
    tell(has_type(Table, hsr_objects:'Table')),
    tell(has_type(Armchair, hsr_objects:'Other')).


%% supporting_surface(?Surface).
%
supporting_surface(SurfaceLink):-
    get_urdf_id(URDF),
    %write(SurfaceLink),
    urdf_link_collision_shape(URDF,SurfaceLink,ShapeTerm,_),
    %write(ShapeTerm),
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

is_surface(Surface) :-
    all_surfaces(Surfaces),
    member(Surface, Surfaces).

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

is_object(Object) :-
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

is_legal_obj_position([X,Y,Z]) :-
    position_supportable_by_surface([X,Y,Z], _). % call from store_obj_info_server


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

