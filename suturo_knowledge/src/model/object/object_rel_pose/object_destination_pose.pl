:- module(object_destination_pose,
          [
              object_destination_pose/3
          ]).

% 10 cm space needed to both sides for the gripper to open completely
robot_gripper_space(0.10).

%% object_destination_pose(+Object, +Options, -PoseStamped) is semidet.
%
object_destination_pose(Object, Options, [Frame, Pos, Rotation]) :-
    % warning, this is still a rough draft.
    (  once(find_place(Object, Options, [Frame, Pos, Rotation]))
    -> true
    ;  ros_error('could not find a valid destination pose for ~w', [Object]), fail).

%% find_place(+Object, +Options, -PoseStamped) is nondet.
%
find_place(Object, _Options, PoseStamped) :-
    object_depth(Object, ObjectDepth),
    (  best_fitting_destination(Object, NextTo, Destination)
    -> (  possible_pose(Destination, NextTo, ObjectDepth, PoseStamped),
          check_for_collision(Destination, PoseStamped)
       ;  ros_info('No collision-free pose for ~w, computing colliding pose', [Object]),
          possible_pose(Destination, NextTo, ObjectDepth, PoseStamped))
    ;  ros_info('No Existing destination location has an object similar to ~w, getting a free destination', [Object]),
       free_destination(Object, Destination),
       free_destination_pose(Destination, ObjectDepth, PoseStamped)
    ;  ros_info('No Free destination, searching for one with a free space'),
       has_type(Object, Type),
       predefined_destination_location(Type, Destination),
       possible_pose(Destination, Destination, ObjectDepth, PoseStamped),
       check_for_collision(Destination, PoseStamped)
    ).

:- rdf_meta(best_fitting_destination(r,-)).

%% best_fitting_destination(+Object, -NextTo, -Destination) is nondet.
%
% search all possible destinations and
% find the one where the objects already there match the best.
% allows backtracking to find the second-best etc destination.
best_fitting_destination(Object, NextTo, Destination) :-
    has_type(Object, Type),
    findall(Obj-Destination,
            (predefined_destination_location(Type, Destination),
             triple(Obj, soma:isOntopOf, Destination)),
            ObjsDests),
    \+ ObjsDests == [],
    pairs_keys(ObjsDests, Objs),
    most_similar_object(Object, Objs, NextTo),
    member(NextTo-Destination,ObjsDests).

free_destination(Object, Destination) :-
    has_type(Object, Type),
    predefined_destination_location(Type, Destination),
    \+ triple(_Obj, soma:isOntopOf, Destination).

free_destination_pose(Furniture, ObjectDepth, [Frame, [X,RightY,0], [0,0,0,1]]) :-
    object_shape_workaround(Furniture, Frame, ShapeTerm, _Pose, _Material),
    ShapeTerm = box(DX,DY,_DZ),
    robot_gripper_space(GripperSpace),
    RightY is -DY/2 + GripperSpace+0.05,
    X is min(-DX/2 + ObjectDepth + 0.05, 0).

possible_pose(Furniture, NextTo, ObjectDepth, [Frame, [X,Y,0], [0,0,0,1]]) :-
    % assuming the frame is on top of the center of the furniture.
    % and assuming the approach direction is from -x.
    object_shape_workaround(Furniture, Frame, ShapeTerm, _Pose, _Material),
    kb_call(is_at(NextTo, [Frame, [_, CenterY, _], _])),
    ShapeTerm = box(DX,DY,_DZ),
    robot_gripper_space(GripperSpace),
    RightY is -DY/2 + GripperSpace+0.05,
    LeftY is -RightY,
    X is min(-DX/2 + ObjectDepth + 0.05, 0),
    MaxY is max(-RightY + CenterY, LeftY - CenterY),
    Delta is GripperSpace/2,
    increasing_alternating(MaxY, Delta, GripperSpace, YDiff),
    Y is CenterY + YDiff,
    Y >= RightY,
    Y =< LeftY.

increasing_alternating(_Max, _Delta, ValueIn, ValueOut) :-
    ValueOut is ValueIn.
increasing_alternating(_Max, _Delta, ValueIn, ValueOut) :-
    ValueOut is -ValueIn.
increasing_alternating(Max, Delta, ValueIn, ValueOut) :-
    ValueMid is ValueIn+Delta,
    Max >= ValueMid,
    increasing_alternating(Max, Delta, ValueMid, ValueOut).

for_loop(Start, End, _Delta, Start) :-
    (  Start =< End
    -> true
    ;  !,fail).
for_loop(Start, End, Delta, Value) :-
    Next is Start + Delta,
    for_loop(Next, End, Delta, Value).

check_for_collision(Furniture, [Frame, [X,Y,Z], _Rot]) :-
    robot_gripper_space(RobotGripperSpace),
    forall(kb_call((triple(Object, soma:isOntopOf, Furniture),
                    is_at(Object, [Frame, [X2,Y2,Z2], _]))),
	   % only the gripper space is needed, as teh object is completely in the gripper
           \+ collides_pos([X,Y,Z], [X2,Y2,Z2], RobotGripperSpace)).

collides_pos([X,Y,Z], [X2,Y2,Z2], Distance) :-
    abs(X-X2) =< Distance,
    abs(Y-Y2) =< Distance,
    abs(Z-Z2) =< Distance.

object_depth(Object, Depth) :-
    object_shape_workaround(Object, _, ShapeTerm, _, _),
    shape_depth(ShapeTerm, Depth),
    !.
% default value
object_depth(_Object, 0.2).

shape_depth(box(Depth,_,_), Depth) :- !.
shape_depth(cylinder(Radius,_), Radius) :- !.
shape_depth(sphere(Radius), Radius) :- !.
% TODO mesh implementation
