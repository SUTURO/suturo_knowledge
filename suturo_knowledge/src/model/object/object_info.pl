%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
        object_rel_pose/3,
        object_rel_pose/4,
        object_pose/2
	  ]).

:- use_module(library('ros/tf/tf'),
	      % actually uses tf:tf_get_pose, but that is not exported by tf
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
%
% Get the pose of an object.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).

%% object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to the object based on the type of relation.
%
% @param Object The Object to which the position is relative to.
% @param Type The type of relation. It can be "perceive", "interact" or "destination".
% @param PoseStamped The position relative to the Object.
%
object_rel_pose(Object, Type, PoseStamped) :-
    object_rel_pose(Object, Type, [], PoseStamped).

%% object_rel_pose(+Object, +Type, +Options, -PoseStamped) is semidet.
%
% See object_rel_pose/3 for basic information.
% This predicate additionally accepts a list of options that might change the behavior of ceratin types.
% 

object_rel_pose(Object, perceive, Options, PoseStamped) :-
    object_perceive_pose(Object, Options, PoseStamped), !.

object_rel_pose(Object, place, Options, PoseStamped) :-
    object_place_pose(Object, Options, PoseStamped), !.

object_rel_pose(Object, Type, _Options, PoseStamped) :-
    % call the appropriate predicate based on the type of the object
    (has_type(Object, soma:'DesignedComponent')
     ->	component_rel_pose(Object, Type, PoseStamped)
    ; has_type(Object, soma:'DesignedContainer')
     ->	container_rel_pose(Object, Type, PoseStamped)
    ; has_type(Object, soma:'DesignedFurniture')
     ->	furniture_rel_pose(Object, Type, PoseStamped)
    ; ros_error('Error: Unknown object class for object_rel_pose: object ~w.', [Object]),
      false
    ).

:- predicate_options(object_perceive_pose/3, 2,
		     [ direction(atom)
		     ]).

%% object_perceive_pose(+Object, +Options, -PoseStamped) is semidet.
%
% get the pose from which the object should be perceived.
object_perceive_pose(Object, Options, [Frame, Pos, Rotation]) :-
    center_pose(Object, Pose, ShapeTerm),
    option(direction(Dir), Options, '-x'),
    dir_size(Dir, ShapeTerm, Size),
    perceive_distance(Object, PerceiveDistance),
    Distance is (Size/2) + PerceiveDistance, %% TODO make this number depend on the target object
    direction_quaternion(Dir, Rotation),
    rel_pose(Dir, Pose, Distance, [Frame, Pos, _]),
    !.

perceive_distance(Object, PerceiveDistance) :-
    kb_call(is_shelf(Object)),
    !,
    % TODO don't hardcode these
    PerceiveDistance = 1.06.

perceive_distance(_Object, 0.67).

:- predicate_options(object_place_pose/3, 2,
		     [ direction(atom),
		       index(float),
		       maxindex(float)
		     ]).

%% object_place_pose(+Object, +Options, -PoseStamped) is semidet.
%
% Get the Pose on a Table where an object should be placed.
% The Options index and maxindex are mandatory.
% If they are not present, this fails.
object_place_pose(Object, Options, [Frame, Pos, Rotation]) :-
    center_pose(Object, Pose, ShapeTerm),
    option(direction(Dir), Options, '-x'),
    % Index and MaxIndex are 1-based
    option(index(Index), Options),
    option(maxindex(MaxIndex), Options),
    % TODO get better distance from furniture edge
    Distance = 0.15,
    rotate_dir(Dir, RotDir),
    dir_size(RotDir, ShapeTerm, Space),
    dir_size(Dir, ShapeTerm, Size),
    ToFront is Size-Distance,
    rel_pose(Dir, Pose, ToFront, FrontPose),
    PartSpace is Space / MaxIndex,
    HalfSpace is -(Space / 2),
    rel_pose(RotDir, FrontPose, HalfSpace, FrontCornerPose),
    Shift is (Index - 0.5) * PartSpace,
    direction_quaternion(Dir, Rotation),
    rel_pose(RotDir, FrontCornerPose, Shift, [Frame, Pos, _]),
    !.

%% center_pose(+Object, -Pose, -ShapeTerm) is semidet.
%
% This predicate returns the shape and the center pose of an Object.
% If the object is of type table, it is assumed that the pose is the front edge center and as that should be moved_to_center/3 d.
center_pose(Object, Pose, ShapeTerm) :-
    %kb_call(object_shape(Object, _, ShapeTerm, Pose, _)),
    % TODO: Fix object_shape
    object_pose(Object, BasePose),
    tmp_object_shape(Object, ShapeTerm),
    (  kb_call(is_table(Object)) % Base Pose is front edge center for tables
    -> move_to_center(BasePose, ShapeTerm, Pose)
    ;  Pose = BasePose),
    !.    

%% move_to_center(+FrontEdgePose, +ShapeTerm, -CenterPose) is semidet.
%
% Move the position by half the size in x direction.
% This is useful to get from the table:front_edge_center to the center of the table.
move_to_center([Frame,[X,Y,Z], Rotation], ShapeTerm, [Frame,[X2,Y,Z], Rotation]) :-
    dir_size('-x', ShapeTerm, Size),
    X2 is X + (Size/2).

%% dir_size(+Dir, +Object, -Size) is semidet.
%
% get the size of an object in a direction
dir_size('-x', box(Size,_,_), Size) :- !.
dir_size('+x', box(Size,_,_), Size) :- !.
dir_size('-y', box(_,Size,_), Size) :- !.
dir_size('+y', box(_,Size,_), Size) :- !.

%% rotate_dir(+DirIn, -DirOut) is semidet.
%% rotate_dir(?DirIn, ?DirOut) is nondet.
%
% rotate DirIn by 90°.
rotate_dir('-x', '-y').
rotate_dir('+x', '+y').
rotate_dir('-y', '+x').
rotate_dir('+y', '-x').

direction_quaternion('+x', [0,0,1,0]).
direction_quaternion('-x', [0,0,0,1]).
direction_quaternion('+y', [0,0,0.7071067811865475,-0.7071067811865475]).
direction_quaternion('-y', [0,0,0.7071067811865475,0.7071067811865475]).

%% rel_pose(+Dir, +PoseIn, +Distance, -PoseOut) is det.
%
% Calculate the PoseOut Distance units away from PoseIn in the direction Dir.
rel_pose(Dir, [Frame,[X,Y,Z],Rotation], Distance, [Frame,PositionOut,Rotation]) :-
    rel_pose0(Dir, [X,Y,Z], Distance, PositionOut).

rel_pose0('-x', [X,Y,Z], Distance, [X2,Y,Z]) :-
    X2 is X - Distance, !.

rel_pose0('+x', [X,Y,Z], Distance, [X2,Y,Z]) :-
    X2 is X + Distance, !.

rel_pose0('-y', [X,Y,Z], Distance, [X,Y2,Z]) :-
    Y2 is Y - Distance, !.

rel_pose0('+y', [X,Y,Z], Distance, [X,Y2,Z]) :-
    Y2 is Y + Distance, !.

%% tmp_object_shape(+Obj, -ShapeTerm) is semidet.
%
% temporary function to get the shape of a furniture
% TODO: change this to use object_shape/5 when it works.
tmp_object_shape(Obj, ShapeTerm) :-
    kb_call(is_table(Obj)),
    !,
    ShapeTerm = box(0.9, 1.4, 0.75).

tmp_object_shape(Obj, ShapeTerm) :-
    kb_call(is_shelf(Obj)),
    !,
    ShapeTerm = box(0.25, 1.0, 1.4).
