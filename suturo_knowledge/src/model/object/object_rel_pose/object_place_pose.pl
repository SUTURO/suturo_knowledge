:- module(object_place_pose,
	  [
	      object_place_pose(r,+,-)
	  ]).

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

