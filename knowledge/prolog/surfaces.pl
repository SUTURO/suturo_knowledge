
:- module(surfaces, % TODO SORT MEEEEEEEEEE
    [
    supporting_surface/1,
    surface_big_enough/1,
    square_big_enough/2,
    object_supportable_by_surface/2,
    position_supportable_by_surface/2,
    assert_surface_types/1,
    assert_object_on/2,
    shelf_surfaces/1, %shelf_floors/1
    big_shelf_surfaces/1,
    table_surfaces/1,
    shelf_floor_at_height/2,
    object_goal_surface/2,
    object_goal_surface/3,
    object_goal_surface/4,
    objects_on_surface/2,
    object_goal_pose_offset/3,
    all_objects_in_whole_shelf/1,
    all_objects_on_tables/1,
    all_objects_on_ground/1,
    all_objects_on_source_surfaces/1,
    all_objects_on_target_surfaces/1,
    all_source_surfaces/1,
    all_target_surfaces/1,
    ground_surface/1,
    place_object/1,
    all_surfaces/1, %replaces all_srdl_objects contains ground
    object_current_surface/2,
    make_all_surface_type_role/2,
    get_surface_role/2,
    make_role/2
    ]).

:- rdf_db:rdf_register_ns(urdf, 'http://knowrob.org/kb/urdf.owl#', [keep(true)]).
:- owl_parser:owl_parse('package://urdfprolog/owl/urdf.owl').

:- rdf_meta % TODO FIX ME
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
    object_goal_pose(r,?,?,?).




/**
*****************************************object - surface relation******************************************************
*/

objects_on_surface(ObjectInstances, SurfaceLink) :-
    findall(ObjectInstance,
        object_current_surface(ObjectInstance, SurfaceLink),
        ObjectInstances).

objects_on_list_of_surfaces(ObjectInstances, SurfaceList):-
    findall(Obj,
    ( 
        member(Surface, SurfaceList),
        objects_on_surface(Objects, Surface),
        member(Obj, Objects)
    ),
        ObjectInstances).

object_current_surface(ObjectInstance, SurfaceLink) :-
    rdf_has(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).

table_surfaces(TableLinks):-
    findall(TableLink, rdf_has(TableLink, hsr_objects:'isSurfaceType',table), TableLinks).

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

ground_surface(GroundSurface):-
    GroundSurface = ground.



all_objects_in_whole_shelf(Instances) :-
    findall(Instance, (
        shelf_surfaces(ShelveLinks),
        member(Shelf, ShelveLinks),
        objects_on_surface(ObjPerShelf, Shelf),
        member(Instance, ObjPerShelf)
        ), Instances).


all_objects_in_gripper(Instances):-
    findall(Instance, (
        objects_on_surface(Objs, gripper),
        member(Instance, Objs)
        ), Instances).



all_objects_on_tables(Instances) :-
    findall(Instance, (
        table_surfaces(TableLinks),
        member(Table, TableLinks),
        objects_on_surface(ObjPerTable, Table),
        member(Instance, ObjPerTable)
        ), Instances).

all_objects_on_ground(Instances) :-
    findall(Instance, (
        ground_surface(Ground),
        objects_on_surface(ObjOnGround, Ground),
        member(Instance, ObjOnGround)
        ), Instances).


assert_object_on(ObjectInstance, SurfaceLink) :-
    all_surfaces(SurfaceLinks),
    member(SurfaceLink,SurfaceLinks),
    kb_retract(ObjectInstance, hsr_objects:'supportedBy', _),
    kb_assert(ObjectInstance, hsr_objects:'supportedBy', SurfaceLink).


%TODO
shelf_floor_at_height(Height, TargetShelfLink) :-
    findall(ShelfFloorLink, (
        big_shelf_surfaces(AllFloorsLinks),
        member(ShelfFloorLink, AllFloorsLinks),
        rdf_urdf_has_child(Joint,ShelfFloorLink),
        joint_abs_position(Joint,[_,_,Z]),
        Z < Height
    ), ShelfFloorsLinks),
    reverse(ShelfFloorsLinks, [TargetShelfLink|_]).

/**
***************************************************find and assert surfaces*********************************************
*/


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



all_surfaces(SurfaceLinks):-
    findall(SurfaceLink,
        rdf_has(SurfaceLink,hsr_objects:'isSurfaceType',_),
        SurfaceLinks
    ).

% Objs is a list of all Objects on all source surfaces.
all_objects_on_source_surfaces(Objs):-
    all_source_surfaces(Surfaces),
    objects_on_list_of_surfaces(Objs, Surfaces).

% Objs is a list of all Objects on all target surfaces.
all_objects_on_target_surfaces(Objs):-
    all_target_surfaces(Surfaces),
    objects_on_list_of_surfaces(Objs, Surfaces).

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

%% supporting_surface(?Surface).
%
supporting_surface(SurfaceLink):-
    rdf_urdf_link_collision(SurfaceLink,ShapeTerm,_),
    surface_big_enough(ShapeTerm),
    true.


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



/**
****************************************find what surface object is on**************************************************
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
*****************************************object_goal_surface***********************************************************
*/

%TODO limit to the big shelf

object_goal_surface(Instance, SurfaceLink) :-
    object_goal_surface(Instance, SurfaceLink, _).

object_goal_surface(Instance, SurfaceLink, Context) :-
    object_goal_surface(Instance, SurfaceLink, Context, _).

% Sort by size if object class is OTHER
object_goal_surface(Instance, SurfaceLink, Context, ShelfObj) :-
    kb_type_of(Instance, hsr_objects:'Other'),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdf_has(Instance, hsr_objects:'size', Size),
    rdf_has(ShelfObj, hsr_objects:'size', Size),
    object_current_surface(ShelfObj, SurfaceLink),
    string_concat('I will put this to the other ', Size, Stringpart1),
    string_concat(Stringpart1, ' object.', Context).

% Sort by color, if object class is OTHER
object_goal_surface(Instance, SurfaceLink, Context, ShelfObj) :-
    kb_type_of(Instance, hsr_objects:'Other'),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdf_has(Instance, hsr_objects:'colour', Color),
    rdf_has(ShelfObj, hsr_objects:'colour', Color),
    object_current_surface(ShelfObj, SurfaceLink),
    string_concat('I will put this to the other ', Color, Stringpart1),
    string_concat(Stringpart1, ' object.', Context).

%% Same obj class
object_goal_surface(Instance, SurfaceLink, Context, ShelfObj) :-
    kb_type_of(Instance, Class),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_individual_of(ShelfObj, Class),
    object_current_surface(ShelfObj, SurfaceLink),
    rdf_split_url(_, CName, Class),
    string_concat('I will put this to the other ', CName, Context).

%% Same direct superclass
object_goal_surface(Instance, SurfaceLink, Context, ShelfObj) :-
    kb_type_of(Instance, Class),
    owl_direct_subclass_of(Class, Super),
    not(rdf_equal(Super, hsr_objects:'Item')),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_individual_of(ShelfObj, Super),
    object_current_surface(ShelfObj, SurfaceLink),
    rdf_split_url(_, CName, Super),
    string_concat('I will put this to the similar ', CName, Context).

%% Same superclass 2 levels up
object_goal_surface(Instance, SurfaceLink, Context, ShelfObj) :-
    kb_type_of(Instance, Class),
    owl_direct_subclass_of(Class, Super),
    not(rdf_equal(Super, hsr_objects:'Item')),
    owl_direct_subclass_of(Super, Supersuper),
    not(rdf_equal(Supersuper, hsr_objects:'Item')),
    all_objects_in_whole_shelf(ShelfObjs),
    member(ShelfObj, ShelfObjs),
    rdfs_individual_of(ShelfObj, Supersuper),
    object_current_surface(ShelfObj, SurfaceLink),
    rdf_split_url(_, CName, Supersuper),
    string_concat('I will put this to the somehow similar ', CName, Context).


%% If there is no corresponding class, take some shelf in the middle
object_goal_surface(Instance, SurfaceLink, Context, Self) :-
    (shelf_floor_at_height(0.9, SurfaceLink);
    shelf_floor_at_height(0.6, SurfaceLink)),
    objects_on_surface([], SurfaceLink),
    Self = Instance,
    Context = 'I will create a new group for this'.

%% If middle shelves also occupied, take rest (lowest level first). WARNING: HSR may not be able to reach upper levels
object_goal_surface(Instance, SurfaceLink, Context, Self) :-
    big_shelf_surfaces(ShelfFloors),
    member(SurfaceLink,ShelfFloors),
    objects_on_surface([], SurfaceLink),
    Self = Instance,
    Context = 'I will create a new group for this'.

offsets(Offset) :-
    Offset = [0, -0.05, 0.05, -0.1, 0.1, -0.15, 0.15, -0.2, 0.2, -0.25, 0.25, -0.3, 0.3, 0.35, 0.35].




/**
*********************************************object_goal_pose***********************************************************
*/

object_goal_pose_offset(Instance, [[X,Y,Z], Rotation],Context):-
    object_goal_pose(Instance, [[X,Y,OZ], Rotation],Context),
    object_dimensions(Instance,_,_,H),
    Z is OZ + H/2 + 0.03.



object_goal_pose(Instance, [Translation, Rotation]) :-
    object_goal_pose(Instance, [Translation, Rotation], _).

object_goal_pose(Instance, [Translation, Rotation], Context) :-
    object_goal_pose(Instance, [Translation, Rotation], Context, _).

%% In case a reference group in the shelf is found
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface(Instance, Surface, Context, RefObject),
    not(rdf_equal(Instance, RefObject)),
    surface_pose_in_map(Surface, [_, Rotation]),
    rdf_has(RefObject, hsr_objects:'inGroup', Group),
    group_mean_pose(Group, [X,Y,Z], _),
    offsets(Offset),
    member(XOffset, Offset),
    hsr_lookup_transform(map, 'iai_kitchen/shelf_left_side_piece', [LeftBorder,_,_], _),
    hsr_lookup_transform(map, 'iai_kitchen/shelf_right_side_piece', [RightBorder,_,_], _),
    NewX is X + XOffset,
    NewX < LeftBorder - 0.1,
    NewX > RightBorder + 0.1,
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z] ,!.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface(Instance, Surface, Context, RefObject),
    rdf_equal(Instance, RefObject),
    surface_pose_in_map(Surface, [[X,Y,Z], Rotation]),
    offsets(Offset),
    member(XOffset, Offset),
    NewX is X + XOffset,
    not(hsr_existing_object_at([map,_,[NewX, Y, Z + 0.1], Rotation], 0.2, _)),
    Translation = [NewX, Y, Z].
