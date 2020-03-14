
:- module(surfaces,
    [
    assert_surface_types/1,
    supporting_surface/1,
    assert_object_on/2,
    surface_type_of/2,
    %% FIND SURFACES
    all_surfaces/1, %replaces all_srdl_objects contains ground
    all_source_surfaces/1,
    all_target_surfaces/1,
    get_surface_id_by_name/2,
    ground_surface/1,
    shelf_surfaces/1, 
    big_shelf_surfaces/1, % will soon be deprecated
    shelf_floor_at_height/2, % will soon be deprecated
    table_surfaces/1, 
    object_current_surface/2,
    select_surface/2,
    %% FIND OBJs
    objects_on_surface/2,
    all_objects_on_source_surfaces/1,
    all_objects_on_target_surfaces/1,
    all_objects_on_ground/1,
    all_objects_in_whole_shelf/1, % will soon be deprecated
    all_objects_on_tables/1,
    %% CREATE OBJECT
    place_object/1,
    object_supportable_by_surface/2,
    position_supportable_by_surface/2,
    %% ROLES
    make_ground_source/0,
    make_all_shelves_target/0,
    make_all_tables_source/0,
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
    object_supportable_by_surface(r,r),
    position_supportable_by_surface(r,r),
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



/**
* is called in init.pl
*/
assert_surface_types(SurfaceLink):-
    rdf_assert(ground,hsr_objects:'isSurfaceType',ground),
    supporting_surface(SurfaceLink),
    rdf_urdf_name(SurfaceLink,Name),
    ( sub_string(Name,_,_,_,shelf)
    ->rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',shelf)
    ;
    ( sub_string(Name,_,_,_,table)
    ->rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',table)
    ; rdf_assert(SurfaceLink,hsr_objects:'isSurfaceType',other)
    )).


%% supporting_surface(?Surface).
%
supporting_surface(SurfaceLink):-
    rdf_urdf_link_collision(SurfaceLink,ShapeTerm,_),
    surface_big_enough(ShapeTerm),
    true.

get_surface_id_by_name(Name, SurfaceId):-
    (rdf_urdf_name(SurfaceId, Name), all_surfaces(Surfaces), member(SurfaceId, Surfaces)
        -> true
        ;  (Name = ground
            -> SurfaceId = ground
            ; false
        )
    ).

forget_objects_on_surface(SurfaceLink) :-
    objects_on_surface(Objs,SurfaceLink),
    member(Obj,Objs),
    hsr_forget_object(Obj).


surface_big_enough(box(X, Y, _)):- %TODO Support other shapes
    square_big_enough(X,Y).

%surface_big_enough(mesh(_,[X,Y,_])):- % TODO? Support Meshes (They are all asserted with size 1, 1, 1)
%    square_big_enough(X,Y).

square_big_enough(X,Y):- %TODO Support other shapes
    Size = X * Y,
    (  Size >= 0.09 , X > 0.2, Y > 0.2
    -> true
    ; fail
    ).


assert_object_on(ObjectInstance, SurfaceLink) :-
    all_surfaces(SurfaceLinks),
    member(SurfaceLink,SurfaceLinks),
    kb_retract(ObjectInstance, hsr_objects:'supportedBy', _),
    kb_assert(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).


surface_type_of(Surface, Type):-
    rdf_has(Surface, hsr_objects:'isSurfaceType', Type).


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


big_shelf_surfaces(ShelfLinks) :-
    findall(ShelfLink,
    (
        rdf_has(ShelfLink, hsr_objects:'isSurfaceType',shelf),
        rdf_urdf_name(ShelfLink,Name),
        not(sub_string(Name,_,_,_,small))
    ),
    ShelfLinks).


% Deprecated
shelf_floor_at_height(Height, TargetShelfLink) :-
    findall(ShelfFloorLink, (
        big_shelf_surfaces(AllFloorsLinks),
        member(ShelfFloorLink, AllFloorsLinks),
        rdf_urdf_has_child(Joint,ShelfFloorLink),
        joint_abs_position(Joint,[_,_,Z]),
        Z < Height
    ), ShelfFloorsLinks),
    reverse(ShelfFloorsLinks, [TargetShelfLink|_]).


table_surfaces(TableLinks):-
    findall(TableLink, rdf_has(TableLink, hsr_objects:'isSurfaceType',table), TableLinks).


object_current_surface(ObjectInstance, SurfaceLink) :-
    rdf_has(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).


/**
*****************************************CREATE OBJECTS******************************************************
*/

objects_on_surface(ObjectInstances, SurfaceLink) :-
    findall(ObjectInstance,
        object_current_surface(ObjectInstance, SurfaceLink),
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


all_objects_in_whole_shelf(Instances) :-
    findall(Instance, (
        shelf_surfaces(ShelveLinks),
        member(Shelf, ShelveLinks),
        objects_on_surface(ObjPerShelf, Shelf),
        member(Instance, ObjPerShelf)
        ), Instances).


all_objects_on_tables(Instances) :-
    findall(Instance, (
        table_surfaces(TableLinks),
        member(Table, TableLinks),
        objects_on_surface(ObjPerTable, Table),
        member(Instance, ObjPerTable)
        ), Instances).


all_objects_in_gripper(Instances):-
    findall(Instance, (
        objects_on_surface(Objs, gripper),
        member(Instance, Objs)
        ), Instances).



select_surface([X,Y,Z], Surface) :-
    (  position_supportable_by_surface([X,Y,Z], Surface1)
    -> Surface = Surface1
    ;  (Z < 0.5
         -> Surface = ground
         ;  false
       )
    ).



/**
*****************************************FIND OBJs******************************************************
*/


%%
% finds the surface an object was seen on. When there is no surface supporting the object and
% the center point of the object < 0.5 the object is placed on the ground. Otherwise the query resolves to false.
place_object(Object):-
    (  object_supportable_by_surface(Object, Surface)
    -> assert_object_on(Object,Surface)
    ;  object_pose(Object,[_,_,[_,_,Z],_]),
       Z < 0.5,
       assert_object_on(Object,ground)
    ).


% finds and returns surfaces the object might be standing on.
object_supportable_by_surface(Object, SurfaceLink):-
    all_surfaces(SurfaceLinks),
    member(SurfaceLink,SurfaceLinks),
    object_pose(Object,[_,_,[X,Y,Z],_]),
    position_supportable_by_surface([X,Y,Z], SurfaceLink).


position_supportable_by_surface([X,Y,Z], SurfaceLink):-
    rdf_urdf_link_collision(SurfaceLink,ShapeTerm,_),
    rdf_urdf_has_child(Joint, SurfaceLink),
    position_above_surface(Joint, ShapeTerm, [X,Y,Z], Distance),
    ( (Distance < 0.25, Distance > -0.05) % TODO Find a good threshold
    -> true
    ; fail).


%% Joint supporting the ShapeTerm, the ShapeTerm, Position of the object, return Distance vertical distance
position_above_surface(Joint, ShapeTerm, [X,Y,Z], Distance):-
    joint_abs_position(Joint,[JPosX,JPosY,JPosZ]),
    joint_abs_rotation(Joint,[Roll,Pitch,Yaw]),
     %TODO THIS IS A STUPID WORK AROUND. MAKE THE URDF MORE CONSISTANT
    (rdf_urdf_name(Joint,Name),sub_string(Name,_,_,_,center)
    -> JPosZR is JPosZ *2
    ; JPosZR is JPosZ
    ),
    ( point_on_surface([JPosX, JPosY, _], [Roll,Pitch,Yaw], ShapeTerm, [X,Y,_])
    -> Distance is Z - JPosZR, true
    ; fail
    ).

/**
***************************************************ROLES*********************************************
*/

make_ground_source:-
    make_all_surface_type_role(ground, source).


make_all_shelves_target:-
    make_all_surface_type_role(shelf, target).


make_all_tables_source:-
    make_all_surface_type_role(table, source).

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