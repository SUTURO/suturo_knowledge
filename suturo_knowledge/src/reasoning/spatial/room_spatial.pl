:- module(room_spatial,
          [ check_inside_room(r,r),
            check_position_inside_room(+,r)
          ]).

:- use_module(library('utility/algebra'), [transform_multiply/3]).

:- dynamic waited/1.

%% check_inside_room(+Object, +Room) is semidet.
%
% checks if an object is inside of a room.
check_inside_room(Object,Room) :-
    object_shape_workaround(Room, Frame, ShapeTerm, _, _),
    ShapeTerm = box(DX,DY,_),
    (waited(time)
    -> true
    ; sleep(0.2), assert(waited(time))
    ),
    (kb_call(is_at(Object, [Frame, [X,Y,_], _]))
    -> true
    ;  ros_error('[check_inside_room] is_at failed. Increase the wait delay before the first call to hotfix this!'),
       fail
    ),
    % Assuming z is up direction,
    % X and Y have to be inside the area.
    % so between center + diameter / 2 and center - diameter / 2.
    % center is at 0.
    X =< + DX/2,
    X >= - DX/2,
    Y =< + DY/2,
    Y >= - DY/2.

%% check_position_inside_room(+PoseStamped, +Room) is semidet.
%
% checks if a PoseStamped is inside a room.
% The rotation part of the poseStamped is not used, so feel free to set it to _ (the anonymous variable).
check_position_inside_room([Frame, Pos, _Rot], Room) :-
    object_shape_workaround(Room, RoomFrame, ShapeTerm, _, _),
    ShapeTerm = box(DX,DY,_),
    tf_transform_point(Frame, Pos, RoomFrame,[X,Y,_]),
    X =< + DX/2,
    X >= - DX/2,
    Y =< + DY/2,
    Y >= - DY/2.

%% tf_transform_point(+OldFrame, +OldPos, +NewFrame, -NewPos) is det.
%
% transform a point from one reference frame to another.
tf_transform_point(OldFrame, OldPos, NewFrame, NewPos) :-
    (  OldFrame == map
    %% Trying to use is_at with map as the OldFrame wont work, so use the inverse.
    -> kb_call(is_at(NewFrame, [OldFrame | InvTransform])),
       transform_invert([OldFrame, NewFrame | InvTransform],
                        [NewFrame, OldFrame | Transform])
    ;  kb_call(is_at(OldFrame, [NewFrame | Transform]))
    ),
    transform_multiply([NewFrame, OldFrame | Transform],
                       [OldFrame, target, OldPos, [0,0,0,1]],
                       [NewFrame, target, NewPos, _]).
