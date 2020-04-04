:- module(spatial_comp,
    [
        hsr_lookup_transform/4,
        hsr_existing_object_at/3,
        surface_pose_in_map/2,
        object_supportable_by_surface/2,
        position_supportable_by_surface/2,
        %joint_abs_position/2, % deprecated
        %joint_abs_rotation/2, % deprecated
        %point_in_rectangle/5,
        %point_on_surface/4,
        distance_to_robot/2,
        %find_corners/4,
        % Type cast
        object_frame/2,
        surface_frame/2
    ]).

:- rdf_meta
    hsr_lookup_transform(r,r,?,?),
    hsr_existing_object_at(r,r,?),
    joint_abs_position(r,?),
    quaternion_to_euler(r,?),
    euler_to_quaternion(r,?).



hsr_lookup_transform(SourceFrame, TargetFrame, Translation, Rotation) :-
    tf_lookup_transform(SourceFrame, TargetFrame, pose(Translation,Rotation)).
%    tf_lookup_transform(SourceFrame, TargetFrame, PoseTerm),
%    owl_instance_from_class(knowrob:'Pose', [pose=PoseTerm], Pose),
%    transform_data(Pose,(Translation, Rotation)).

hsr_existing_object_at(Pose, Threshold, Instance) :-
    rdf(Instance, rdf:type, owl:'NamedIndividual', belief_state),
    rdfs_individual_of(Instance, hsr_objects:'Item'),
    object_pose(Instance, OldPose),
    transform_close_to(Pose, OldPose, Threshold).


surface_pose_in_map(SurfaceLink, Pose) :-
    surface_frame(SurfaceLink, Frame),
    hsr_lookup_transform(map, Frame, Translation, Rotation),
    Pose = [Translation, Rotation].

%%%%%%%%%%%%%%%  Supportable by surface  %%%%%%%%%%%%%%%%%5

% finds and returns surfaces the object might be standing on.
object_supportable_by_surface(Object, Surface):-
    all_surfaces(Surfaces),
    member(Surface,Surfaces),
    surface_front_edge_center_frame(Surface, Frame),
    hsr_lookup_transform(Frame, Object, Position, _),
    relative_position_supportable_by_surface(Position, Surface).
    
object_supportable_by_surface(Object, ground):-
    hsr_lookup_transform(map, Object, [_,_,Z], _),
    position_supportable_by_ground(Z).

position_supportable_by_surface(Position, Surface) :-
    all_surfaces(Surfaces),
    member(Surface, Surfaces),
    surface_front_edge_center_frame(Surface, Frame),
    tf_transform_point(map, Frame, Position, RelativePosition),
    relative_position_supportable_by_surface(RelativePosition, Surface).
    
position_supportable_by_surface(Position, ground) :-
    position_supportable_by_ground(Position).

relative_position_supportable_by_surface([X,Y,Z],Surface) :-
    rdf_urdf_link_collision(Surface, box(Width, Depth, _), _),
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= Z,
    ThBelow =< Z,
    Width/2 >= abs(X),
    Depth >= (-1) * Y.

position_supportable_by_ground(ZPos) :-
    threshold_surface(ThAbove, ThBelow),
    ThAbove >= ZPos,
    ThBelow =< ZPos.

position_supportable_by_ground([_,_,Z]) :-
    position_supportable_by_ground(Z).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%surface_pose_in_map(SurfaceLink, [[PX,PY,PZR], [X,Y,Z,W]]) :-
%    rdf_urdf_has_child(Joint,SurfaceLink),
%    joint_abs_position(Joint,[PX,PY,PZ]),
%    %TODO THIS IS A WORKAROUND. MAKE THE URDF MORE CONSISTANT
%    (rdf_urdf_name(SurfaceLink,Name),sub_string(Name,_,_,_,center)
%    -> PZR is PZ *2
%    ; PZR is PZ
%    ),
%    joint_abs_rotation(Joint,[Roll,Pitch,Yaw]),
%    euler_to_quaternion([Roll,Pitch,Yaw], [X,Y,Z,W]).



%Manually tested using RViz
%used to find corners of a surface
% calculates the position of a a joint relative to map;
%Iterating over all of its parents and adding their relativ position.
%joint_abs_position(Joint,[PosX,PosY,PosZ]) :-
%  rdf_has(_,'http://knowrob.org/kb/urdf.owl#hasRootLink',RootLink),
%  (  not(rdf_urdf_has_parent(Joint, RootLink)) % rdf_urdf_has_parent(Joint, _),
%  -> rdf_urdf_has_parent(Joint, Link), rdf_urdf_has_child(SubJoint,Link),
%       rdf_urdf_joint_origin(Joint,[_,_,[JPosX,JPosY,JPosZ],_]),
%       rdf_urdf_joint_origin(SubJoint,[_,_,_,[SQuatX,SQuatY,SQuatZ,SQuatW]]),
%       joint_abs_position(SubJoint,[SPosX,SPosY,SPosZ]),
%       quaternion_to_euler([SQuatX,SQuatY,SQuatZ,SQuatW],[Roll,Pitch,Yaw]),
%       rotate_around_axis(x,Roll,[JPosX,JPosY,JPosZ],[NX1,NY1,NZ1]),
%       rotate_around_axis(y,Pitch,[NX1,NY1,NZ1],[NX2,NY2,NZ2]),
%       rotate_around_axis(z,Yaw,[NX2,NY2,NZ2],[NX,NY,NZ]),
%       PosX is SPosX + NX,
%       PosY is SPosY + NY,
%       PosZ is SPosZ + NZ
%  ;  mem_retrieve_triple(Joint,urdf:hasOrigin,Origin),
%     transform_data(Origin,([PosX,PosY,PosZ],_))
%  ).


% deprecated
joint_abs_position(Joint, Pos) :-
    rdf_urdf_has_child(Joint, Frame),
    surface_pose_in_map(Frame, [Pos, _]).

%used to find corners of a surface
%joint_abs_rotation(Joint,[Roll,Pitch,Yaw]) :-
%    rdf_has(_,'http://knowrob.org/kb/urdf.owl#hasRootLink',RootLink),
%  (  not(rdf_urdf_has_parent(Joint, RootLink)) % rdf_urdf_has_parent(Joint, _),
%  -> rdf_urdf_has_parent(Joint, Link), rdf_urdf_has_child(SubJoint,Link),
%       rdf_urdf_joint_origin(Joint,[_,_,_,[QuatX,QuatY,QuatZ,QuatW]]),
%       quaternion_to_euler([QuatX,QuatY,QuatZ,QuatW],[JR,JP,JY]),
%       joint_abs_rotation(SubJoint,[SRoll,SPitch,SYaw]),
%       Roll  is JR + SRoll,
%       Pitch is JP + SPitch,
%       Yaw   is JY + SYaw
%  ;  rdf_urdf_joint_origin(Joint,[_,_,_,[QuatX,QuatY,QuatZ,QuatW]]),
%     quaternion_to_euler([QuatX,QuatY,QuatZ,QuatW],[Roll,Pitch,Yaw])
%  ).

% deprecated
joint_abs_rotation(Joint, [Roll, Pitch, Yaw]) :-
    rdf_urdf_has_child(Joint, Frame),
    surface_pose_in_map(Frame, [_, Rot]),
    quaternion_to_euler(Rot, [Roll, Pitch, Yaw]).

% Origin contains Centerpoint and Rotation of the Object
%point_on_surface([PosX, PosY, _], [Roll,Pitch,Yaw], box(X, Y, Z), [XP,YP,_]) :-
%    find_corners([PosX,PosY,_], [Roll,Pitch,Yaw], box(X,Y,Z), [[X1,Y1], [X2,Y2], [X3,Y3], [X4,Y4]]),
%    point_in_rectangle([X1,Y1], [X2,Y2], [X3,Y3], [X4,Y4], [XP,YP]).


%% find_corners(Position, EulerRotation, ShapeTerm, [X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]) || (r,r,r, ?)
%find_corners([PosX,PosY,_], [Roll, Pitch, Yaw], box(X, Y, Z), [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]]):- % Position is the center of the Box|Axis: Roll = X, Pitch = Y, Yaw = Z
%    rotate_around_axis(x,Roll,[X,Y,Z],[NX1,NY1,NZ1]),
%    rotate_around_axis(y,Pitch,[NX1,NY1,NZ1],[NX2,NY2,NZ2]),
%    rotate_around_axis(z,Yaw,[NX2,NY2,NZ2],[NX,NY,_]),
%    X1 is PosX - NX/2,
%    Y1 is PosY - NY/2,
%    X2 is PosX + NX/2,
%    Y2 is PosY - NY/2,
%    X3 is PosX - NX/2,
%    Y3 is PosY + NY/2,
%    X4 is PosX + NX/2,
%    Y4 is PosY + NY/2.


find_corners(Surface, Corners) :-
    rdf_urdf_link_collision(Surface, box(Width, Depth, _), _),
    X1 is (-1)*(Width/2), % front left
    Y1 is 0,
    X2 is X1, % back left
    Y2 is (-1)*Depth,
    X3 is Width/2, % back right
    Y3 is Y2,
    X4 is X3, %front right
    Y4 is Y1,
    Corners = [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]].


%find_corners(Surface, Corners) :-
 %   surface_pose_in_map(Surface, Pose),


%find_corners(Pos, Quaternion, Shape, Result) :-
%    quaternion_to_euler(Quaternion, Euler),
%    find_corners(Pos, Euler, Shape, Result).


%TODO Function to rotate around all axis

%Tested using an online calculator
%% rotates a given vector around the X axis by a given radian angle
%rotate_around_axis(x,Alpha,[X,Y,Z],[X1,Y1,Z1]):- % Alpha is in Radian
%    X1 is X,
%    Y1 is Y * cos(Alpha) - Z * sin(Alpha),
%    Z1 is Y * sin(Alpha) + Z * cos(Alpha).
%%% rotates a given vector around the Y axis by a given radian angle
%rotate_around_axis(y,Alpha,[X,Y,Z],[X1,Y1,Z1]):-
%    X1 is X * cos(Alpha) + Z * sin(Alpha),
%    Y1 is Y,
%    Z1 is (0 - X) * sin(Alpha) + Z* cos(Alpha).
%%% rotates a given vector around the Z axis by a given radian angle
%rotate_around_axis(z,Alpha,[X,Y,Z],[X1,Y1,Z1]):-
%    X1 is X * cos(Alpha) - Y * sin(Alpha),
%    Y1 is X * sin(Alpha) + Y * cos(Alpha),
%    Z1 is Z.


% Determines if a 2D point is inside a 2D rectangle points are represented as [floatx, floaty]
%point_in_rectangle(P1,P2,P3,P4,PX):-
%    size_of_triangle(P1,P4,PX,Size1),
%    size_of_triangle(P3,P4,PX,Size2),
%    size_of_triangle(P2,P3,PX,Size3),
%    size_of_triangle(P1,P2,PX,Size4),
%    SumOfTria is Size1 + Size2 + Size3 + Size4,
%    size_of_triangle(P1,P2,P3,SizeRec2),
%    SizeOfRec is SizeRec2 * 2,
%    SumOfTria < SizeOfRec.

% calulates the size of a triangle represented as 3 points in 2D space.
%size_of_triangle([AX,AY],[BX,BY],[CX,CY],Size):-
%   Size is abs((AX * (BY - CY) + BX * (CY - AY) + CX * (AY - BY)) / 2).



%%
% Calculates the euler representation of a given quaternion rotation
% based on https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Source_Code_2
%quaternion_to_euler([X, Y, Z, W], [Roll, Pitch, Yaw])  :- % Axis: Roll = X, Pitch = Y, Yaw = Z
%    % roll (x-axis rotation)
%    SINR_COSP is 2.0 * ((W * X) + (Y * Z)),
%    COSR_COSP is 1.0 - 2.0 * ((X * X) + (Y * Y)),
%    Roll is atan2(SINR_COSP,COSR_COSP),

    % pitch (y-axis rotation)
%    SINP is 2.0 * ((W * Y) - (Z * X)),
%    SINP_ABS is abs(SINP),
%    INP1 is pi / 2,
%    ( SINP_ABS >= 1
%        -> Pitch is copysign(INP1, SINP)
%        ; Pitch is asin(SINP)
%    ),

    % yaw (z-achis rotation)
%    SINY_COSP is 2.0 * ((W * Z) + (X * Y)),
%    COSY_COSP is 1.0 - 2.0 * ((Y * Y) + (Z * Z)),
%    Yaw is atan2(SINY_COSP, COSY_COSP).

%euler_to_quaternion([Roll, Pitch, Yaw], [X, Y, Z, W]) :-
%    W is cos(Yaw/2) * cos(Pitch/2) * cos(Roll/2) - sin(Yaw/2) * sin(Pitch/2) * sin(Roll/2),
%    X is sin(Yaw/2) * sin(Pitch/2) * cos(Roll/2) + cos(Yaw/2) * cos(Pitch/2) * sin(Roll/2),
%    Y is sin(Yaw/2) * cos(Pitch/2) * cos(Roll/2) + cos(Yaw/2) * sin(Pitch/2) * sin(Roll/2),
%    Z is cos(Yaw/2) * sin(Pitch/2) * cos(Roll/2) - sin(Yaw/2) * cos(Pitch/2) * sin(Roll/2).


distance_to_robot(Obj, Distance) :-
    object_frame(Obj, Frame),
    hsr_lookup_transform(map, Frame, [OX, OY, OZ], _),
    hsr_lookup_transform(map,base_footprint,[BX,BY,BZ],_),
    writeln(([OX,OY,OZ],[BX,BY,BZ])),
    DX is (OX - BX),
    DY is (OY - BY),
    DZ is (OZ - BZ),
    sqrt(((DX*DX) + (DY*DY) + (DZ*DZ)), Distance),
    !.

object_frame(Object, Frame) :-
    split_string(Object, "#", "", [_,ObjFrameString]),
    atom_string(Frame,ObjFrameString).
    
surface_frame(Surface, Frame) :-
    rdf_urdf_name(Surface, Name),
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, Name, Frame).

surface_front_edge_center_frame(Surface, FrontEdgeCenterFrame) :-
    rdf_urdf_name(Surface, FullName),
    split_string(FullName, "_", "", [Name, _]),
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, Name, Part1),
    Suffix = "_front_edge_center",
    atom_concat(Part1, Suffix, FrontEdgeCenterFrame).

