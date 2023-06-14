:- module(object_destination_pose,
          [
              object_destination_pose/3
          ]).

% TODO get updated value from kecks
robot_gripper_space(0.20).

%% object_destination_pose(+Object, +Options, -PoseStamped) is semidet.
%
object_destination_pose(Object, Options, [Frame, Pos, Rotation]) :-
    % warning, this is still a rough draft.
    (  once(find_place(Object, Options, [Frame, Pos, Rotation]))
    -> true
    ;  ros_error('could not find a valid destination pose for ~w', [Object])).

%% find_place(+Object, +Options, -PoseStamped) is nondet.
%
find_place(Object, _Options, PoseStamped) :-
    has_type(Object, Type),
    predefined_destination_location(Type, Destination),
    object_depth(Object, ObjectDepth),
    possible_pose(Destination, ObjectDepth, PoseStamped),
    check_for_collision(Destination, PoseStamped).

possible_pose(Furniture, ObjectDepth, [Frame, [X,Y,0], [0,0,0,1]]) :-
    % assuming the frame is on top of the center of the furniture.
    % and assuming the approach direction is from -x.
    object_shape_workaround(Furniture, Frame, ShapeTerm, _Pose, _Material),
    ShapeTerm = box(DX,DY,_DZ),
    robot_gripper_space(GripperSpace),
    RightY is -DY/2 + GripperSpace/2 + 0.02,
    LeftY is -RightY,
    X is min(-DX + ObjectDepth, 0),
    for_loop(RightY, LeftY, 0.05, Y).

for_loop(Start, End, _Delta, Start) :-
    (  Start =< End
    -> true
    ;  !,fail).
for_loop(Start, End, Delta, Value) :-
    Next is Start + Delta,
    for_loop(Next, End, Delta, Value).

check_for_collision(Furniture, [Frame, [X,Y,Z], _Rot]) :-
    robot_gripper_space(RobotGripperSpace),
    forall((kb_call((triple(Object, soma:isOntopOf, Furniture),
                     is_at(Object, [Frame, [X2,Y2,Z2], _]))),
            object_width(Object, Width)),
           (Distance is RobotGripperSpace+Width,
            \+ collides_pos([X,Y,Z], [X2,Y2,Z2], Distance))).

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


object_width(Object, Depth) :-
    object_shape_workaround(Object, _, ShapeTerm, _, _),
    shape_width(ShapeTerm, Depth),
    !.
% default value
object_width(_Object, 0.2).

shape_width(box(_,Width,_), Width) :- !.
shape_width(cylinder(Radius,_), Radius) :- !.
shape_width(sphere(Radius), Radius) :- !.
% TODO mesh implementation
