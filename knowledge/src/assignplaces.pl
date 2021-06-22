:- module(assignplaces,
    [
      object_goal_pose/4, 
      object_goal_pose/3, % recommendet to be used by Planning
      object_goal_pose/2,
      object_goal_pose_offset_/3
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- rdf_meta
    most_related_class(-,?,?),
    most_related_object(-,?).



%******************** GOAL POSE **********************

object_goal_pose(Instance, [Translation, Rotation]) :-
    object_goal_pose(Instance, [Translation, Rotation], _).

object_goal_pose(Instance, [Translation, Rotation], Context) :-
    object_goal_pose(Instance, [Translation, Rotation], Context, _).

object_goal_pose(Instance, [Translation, Rotation], Context, RefInstance) :-
    object_goal_surface_(Instance, Surface, Context, RefInstance),
    has_bucket_shape(Surface),
    surface_pose_in_map(Surface, [[XWOOffset,Y,Z], Rotation]),
    X is XWOOffset + 0.0,
    % X is XWOOffset - 0.05, for 5cm closer to edge
    % X is XWOOffset + 0.05, for 5cm further away from edge
    % - 0.15 is the outer edge of the current bucket
    Translation = [X,Y,Z],
    !.

object_goal_pose(Instance, [Translation, Rotation], Context, RefInstance) :-
    object_goal_surface_(Instance, Surface, Context, RefInstance),
    object_dimensions(Instance, ObjectWidth, ObjectDepth, _),
    has_table_shape(Surface),
    has_urdf_name(Surface, SurfaceLink),
    min_space_between_objects(MinSpace),
    (not same_as(Instance, RefInstance)
    ->
    (
        triple(RefInstance, hsr_objects:'inGroup', Group),
        tell(triple(Instance, hsr_objects:'inGroup', Group)),
        object_tf_frame(RefInstance, RefObjectFrame),
        tf_lookup_transform(SurfaceLink, RefObjectFrame, pose([RefX, RefY, RefZ], _)),
        object_dimensions(RefInstance, RefObjectWidth, RefObjectDepth, _),
        X is RefX + RefObjectDepth/2 - ObjectDepth/2,
        Y is RefY - RefObjectWidth/2 - ObjectWidth/2 - MinSpace,
        tf_transform_pose(SurfaceLink, 'map', pose([X, Y, 0], [0, 0, 0, 1]), pose(Translation, Rotation))
    );
    (
        all_groups_on_tablelike_surface(Surface, Groups),
        surface_dimensions(Surface, SurfaceWidth, SurfaceDepth, SurfaceHeight),
        (length(Groups, 0)
        ->
        (
            RefX is SurfaceWidth/2 - MinSpace - ObjectDepth/2,
            RefY is SurfaceDepth/2 - MinSpace - ObjectWidth/2,
            tf_transform_pose(SurfaceLink, 'map', pose([RefX, RefY, 0], [0, 0, 0, 1]), pose(Translation, Rotation))
        );
        (
            findall([X, Group],
            (
                member(Group, Groups),
                group_position_on_surface(Group, Surface, [X, _, _])
            ),
            GroupPositions),
            sort(GroupPositions, SortedGroupPositions),
            nth0(0, SortedGroupPositions, [_, FirstGroup]),
            group_position_on_surface(Group, Surface, [GroupX, _, GroupZ]),
            group_dimensions(Group, GroupDepth, _),
            RefX is GroupX - MinSpace - GroupDepth - ObjectDepth/2,
            RefY is SurfaceDepth/2 - MinSpace - ObjectWidth/2,
            tf_transform_pose(SurfaceLink, 'map', pose([RefX, RefY, 0], [0, 0, 0, 1]), pose(Translation, Rotation))
        )),
        tell(has_type(NewGroup, hsr_objects:'Group')),
        tell(triple(Instance, hsr_objects:'inGroup', NewGroup))
    )),
    !.

%% In case a reference group in the shelf is found
object_goal_pose(Instance, [Translation, Rotation], Context, RefObject) :-
    object_goal_surface_(Instance, Surface, Context, RefObject),
    not same_as(Instance, RefObject),
    surface_pose_in_map(Surface, [_, Rotation]),
    triple(RefObject, hsr_objects:'inGroup', Group),
    group_mean_pose(Group, [GroupX,GroupY,GroupZ], _),
    urdf_tf_frame(Surface, SurfaceFrame),
    tf_transform_point(map, SurfaceFrame, [GroupX,GroupY,GroupZ], [_, GroupYOnS, _]),
    offsets(Offset),
    member(YOffset, Offset),
    get_urdf_id(URDF),
    has_urdf_name(Surface, SurfaceLink),
    urdf_link_collision_shape(URDF, SurfaceLink, box(Depth, Width, _), _), % get surface dimensions
    NewY is (GroupYOnS + YOffset),
    NewY < (Width / 2) - 0.1,
    NewY > (Width / -2) + 0.1,
    NewX is - Depth / 2 + 0.03,
    tf_transform_point(Surface, map, [NewX, NewY, 0], [AbsX, AbsY,AbsZ]),
    object_dimensions(Instance,_,_,ObjHeight),
    AbsZOffset is AbsZ + ObjHeight/2 + 0.07,
    not(hsr_existing_object_at_thr([AbsX, AbsY, AbsZOffset], 0.15)),
    Translation = [AbsX, AbsY, AbsZOffset],
    !.

%% When a new group is opened the RefObject is equal to the Instance
object_goal_pose(Instance, [Translation, Rotation], Context, Instance) :-
    ros_info("object_goal_pose created new Group"),
    object_goal_surface_(Instance, Surface, Context, Instance),
    surface_pose_in_map(Surface, [[SX,SY,SZ], Rotation]),
    urdf_tf_frame(Surface, SurfaceFrame),
    tf_transform_point(map, SurfaceFrame, [SX,SY,SZ], [ _, YOnS,_]),  
    offsets(Offset),
    member(YOffset, Offset),
    get_urdf_id(URDF),
    has_urdf_name(Surface, SurfaceLink),
    urdf_link_collision_shape(URDF, SurfaceLink, box(Depth, Width, _), _), % get surface dimensions
    NewYOnS is YOnS + YOffset,
    NewYOnS < (Width / 2) - 0.1,
    NewYOnS > (Width / -2) + 0.1,
    NewXOnS is - Depth / 2 + 0.03,
    tf_transform_point(SurfaceFrame, map, [NewXOnS, NewYOnS, 0], [AbsX, AbsY,AbsZ]),
    object_dimensions(Instance,_,_,ObjHeight),
    AbsZOffset is AbsZ + ObjHeight/2 + 0.07,
    not(hsr_existing_object_at_thr([AbsX, AbsY,AbsZOffset], 0.15)),
    Translation = [AbsX, AbsY,AbsZOffset],
    !.

%object_goal_pose(_, _, "You haven't defined any target surfaces", _) :-
%    all_target_surfaces([]),
%    roswarn("You haven't defined any target surfaces").


% TODO Rework the offsets
object_goal_pose_offset_(Instance, [[XR,YR,ZR], Rotation],Context):-
    %place_objects,
    object_goal_pose(Instance, [[X,Y,Z], Rotation],Context),
    object_dimensions(Instance,_,_,ObjHeight),
    XR is X + 0,
    YR is Y + 0,
    ZR is Z + ObjHeight/2 + 0.07.

