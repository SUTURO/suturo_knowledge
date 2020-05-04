
:- module(surfaces,
    [
    assert_surface_types/1,
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
    all_source_surfaces/1,
    all_target_surfaces/1,
    ground_surface/1,
    shelf_surfaces/1, 
    big_shelf_surfaces/1, % will soon be deprecated
    shelf_floor_at_height/2, % will soon be deprecated
    table_surfaces/1, 
    bucket_surfaces/1,
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
    get_surface_role/2,
    %% FUNCTIONS
    forget_objects_on_surface/1
    ]).

:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).
:- owl_parser:owl_parse('package://urdfprolog/owl/urdf.owl').

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



assert_surface_types(SurfaceLink):-
    rdf_assert(ground,hsr_objects:'isSurfaceType',ground),
    supporting_surface(SurfaceLink),
    rdf_urdf_name(SurfaceLink,Name),
    ( sub_string(Name,_,_,_,shelf)
    ->rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',shelf)
    ;
    ( sub_string(Name,_,_,_,table)
    ->rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',table)
    ;
    ( sub_string(Name,_,_,_,bucket)
    ->rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',bucket)
    ;rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',other))
    )).


%% supporting_surface(?Surface).
%
supporting_surface(SurfaceLink):-
    rdf_urdf_link_collision(SurfaceLink,ShapeTerm,_),
    surface_big_enough(ShapeTerm).


forget_objects_on_surface(SurfaceLink) :-
    objects_on_surface(Objs,SurfaceLink),
    member(Obj,Objs),
    hsr_forget_object(Obj).


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
assert_object_on(ObjectInstance, SurfaceLink) :- % has not been tested yet.
    all_surfaces(SurfaceLinks), % this makes sure, we actually have a surface here
    member(SurfaceLink,SurfaceLinks),
    kb_retract(ObjectInstance, hsr_objects:'supportedBy', _),
    kb_assert(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).


surface_type_of(Surface, Type):- % has not been tested yet.
    rdf_has(Surface, hsr_objects:'isSurfaceType', Type).

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

/**
*****************************************FIND SURFACES******************************************************
*/

all_surfaces(SurfaceLinks):-
    findall(SurfaceLink,
        rdf_has(SurfaceLink,hsr_objects:'isSurfaceType',_),
        SurfaceLinks
    ).


% Surfaces is a list of all SurfaceLinks that are source
all_source_surfaces(Surfaces):-
    all_surfaces(ExistingSurfaces),
    findall(Surface,
    (
        member(Surface, ExistingSurfaces),
        rdf_has(Surface, hsr_objects:'sourceOrTarget', source)
    ),
        Surfaces).


% Surfaces is a list of all SurfaceLinks that are target
all_target_surfaces(Surfaces):-
    all_surfaces(ExistingSurfaces),
    findall(Surface,
    (
        member(Surface, ExistingSurfaces),
        rdf_has(Surface, hsr_objects:'sourceOrTarget', target)
    ),
        Surfaces).


ground_surface(GroundSurface):-
    GroundSurface = ground.


shelf_surfaces(ShelfLinks):-
    findall(ShelfLink, rdf_has(ShelfLink, hsr_objects:'isSurfaceType',shelf),ShelfLinks).


big_shelf_surfaces(ShelfLinks) :- % has not been tested yet.
    findall(ShelfLink,
    (
        rdf_has(ShelfLink, hsr_objects:'isSurfaceType',shelf),
        rdf_urdf_name(ShelfLink,Name),
        not(sub_string(Name,_,_,_,small))
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
    findall(TableLink, rdf_has(TableLink, hsr_objects:'isSurfaceType',table), TableLinks).

bucket_surfaces(BucketLinks):-
    findall(BucketLink, rdf_has(BucketLink, hsr_objects:'isSurfaceType',bucket), BucketLinks).

find_supporting_surface(Object, Surface) :-
    rdf_has(Object, hsr_objects:'supportedBy', Surface).

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
    compare(Order, Dist1, Dist2).


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
    position_supportable_by_surface([X,Y,Z], _).



/**
*****************************************CREATE OBJs******************************************************
*/


%%
% finds the surface an object was seen on. When there is no surface supporting the object and
% the center point of the object < 0.5 the object is placed on the ground. Otherwise the query resolves to false.
place_object(Object):-
    object_supportable_by_surface(Object, Surface),
    assert_object_on(Object,Surface).


%% Joint supporting the ShapeTerm, the ShapeTerm, Position of the object, return Distance vertical distance
position_above_surface(Joint, ShapeTerm, [X,Y,Z], Distance):-
    joint_abs_position(Joint,[JPosX,JPosY,JPosZ]),
    joint_abs_rotation(Joint,[Roll,Pitch,Yaw]),
    ( point_on_surface([JPosX, JPosY, _], [Roll,Pitch,Yaw], ShapeTerm, [X,Y,_])
    -> Distance is Z - JPosZ, true
    ; fail
    ).

/**
***************************************************ROLES*********************************************
*/

make_surfaces_source(Surfaces):-
    forall(member(Surface, Surfaces), make_role(Surface, source)).

make_surfaces_target(Surfaces):-
    forall(member(Surface, Surfaces), make_role(Surface, target)).


% Gives all surfaces with given name (ground, table or shelf) the Role (target or source)
make_all_surface_type_role(SurfaceType, Role):-
    SurfaceType = ground,
    make_role(SurfaceType, Role).

make_all_surface_type_role(SurfaceType, Role):-
    forall(rdf_has(SurfaceLink, hsr_objects:'isSurfaceType',SurfaceType), make_role(SurfaceLink,Role)).


% Gives the gives SurfaceLink the Role (target or source)
make_role(SurfaceLink, Role):-
    rdf_retractall(SurfaceLink, hsr_objects:'sourceOrTarget',_),
    rdf_assert(SurfaceLink, hsr_objects:'sourceOrTarget', Role).


% Role is the role (target or source) of the given SurfaceLink
get_surface_role(SurfaceLink, Role):-
    rdf_has(SurfaceLink, hsr_objects:'sourceOrTarget', Role).