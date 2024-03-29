:- module(object_perceive_pose,
	  [
	      object_perceive_pose(r,+,-)
	  ]).

:- use_module(rel_pose_util).

:- predicate_options(object_perceive_pose/3, 2,
		     [ direction(atom)
		     ]).

%% object_perceive_pose(+Object, +Options, -PoseStamped) is semidet.
%
% get the pose from which the object should be perceived.
object_perceive_pose(Object, Options, [Frame, Pos, Rotation]) :-
    object_shape_workaround(Object, _, ShapeTerm, _, _),
    object_pose(Object, Pose),
    option(direction(Dir), Options, '-x'),
    dir_size(Dir, ShapeTerm, Size),
    perceive_distance(Object, PerceiveDistance),
    Distance is (Size/2) + PerceiveDistance, %% TODO make this number depend on the target object
    direction_quaternion(Dir, Rotation),
    rel_pose(Dir, Pose, Distance, [Frame, Pos, _]),
    !.

perceive_distance(Object, PerceiveDistance) :-
    ( kb_call(has_type(Object, soma:'Cupboard'))
    ; kb_call(has_type(Object, soma:'Shelf'))),
    has_type(_, suturo:'ServeBreakfast'),
    !,
    % in serve breakfast, the robot doesn't perceive the shelf,
    % but just places stuff on top of it.
    % to make placing faster, go nearer to it.
    PerceiveDistance = 0.7.

perceive_distance(Object, PerceiveDistance) :-
    ( kb_call(has_type(Object, soma:'Cupboard'))
    ; kb_call(has_type(Object, soma:'Shelf'))),
    !,
    % TODO don't hardcode these
    PerceiveDistance = 1.19.

perceive_distance(Object, PerceiveDistance) :-
    has_robocup_name(Object, storing_groceries_table),
    !,
    % TODO don't hardcode these
    PerceiveDistance = 0.9.


perceive_distance(_Object, 0.67).
