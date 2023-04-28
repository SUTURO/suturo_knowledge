:- module(rel_pose_util,
	  [
	      fix_direction(+,-),
	      center_pose(r,-,-),
	      move_to_center(+,-,-),
	      dir_size(+,+,-),
	      rotate_dir(+,-),
	      direction_quaternion(+,-),
	      rel_pose(+,+,+,-)
	  ]).

%% fix_direction(+Raw, -Fixed) is semidet
%
% fix the direction if the caller (planning) forgets the singe quotes around it.
fix_direction(Dir,Dir):- atom(Dir), !.
fix_direction(-x,'-x'):- !.
fix_direction(+x,'+x'):- !.
fix_direction(-y,'-y'):- !.
fix_direction(+y,'+y'):- !.

%% center_pose(+Object, -Pose, -ShapeTerm) is semidet.
%
% This predicate returns the shape and the center pose of an Object.
% If the object is of type table, it is assumed that the pose is the front edge center and as that should be moved_to_center/3 d.
center_pose(Object, Pose, ShapeTerm) :-
    object_shape_workaround(Object, _, ShapeTerm, _, _),
    % TODO: Fix object_shape
    object_pose(Object, BasePose),
    %tmp_object_shape(Object, ShapeTerm),
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

%% direction_quaternion(+Dir, -Quaternion) is semidet.
%
% get the rotation quaternion for a direction. The quaternion is the rotation of the robot when approaching a surface in that direction.
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
