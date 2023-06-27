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
    % in case of best_fitting_destination not working properly, comment it
    % and uncomment the next two lines
    best_fitting_destination(Object, Destination),
    %has_type(Object, Type),
    %predefined_destination_location(Type, Destination)
    object_depth(Object, ObjectDepth),
    possible_pose(Destination, ObjectDepth, PoseStamped),
    check_for_collision(Destination, PoseStamped).

:- rdf_meta(best_fitting_destination(r,-)).

%% best_fitting_destination(+Object, -Destination) is nondet.
%
% search all possible destinations and
% find the one where the objects already there match the best.
% allows backtracking to find the second-best etc destination.
best_fitting_destination(Object, Destination) :-
    has_type(Object, Type),
    findall([Destination,BestObject],
            (predefined_destination_location(Type, Destination),
             (  (findall(Obj, triple(Obj, soma:isOntopOf, Destination), Objects),
                 most_similar_object(Object, Objects, BestObject))
             -> true
             ;  BestObject = none)),
            Locations),
    maplist(nth0(1),Locations,Objects),
    sort_by_similarity(Object, Objects, SortedObjects),
    % allow backtracking over the objects, so if one destination is full,
    % the next best can be used.
    member(TargetObject, SortedObjects),
    member([Destination,TargetObject], Locations).

possible_pose(Furniture, ObjectDepth, [Frame, [X,Y,0], [0,0,0,1]]) :-
    % assuming the frame is on top of the center of the furniture.
    % and assuming the approach direction is from -x.
    object_shape_workaround(Furniture, Frame, ShapeTerm, _Pose, _Material),
    ShapeTerm = box(DX,DY,_DZ),
    robot_gripper_space(GripperSpace),
    RightY is -DY/2 + GripperSpace+0.05,
    LeftY is -RightY,
    X is min(-DX/2 + ObjectDepth + 0.05, 0),
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
