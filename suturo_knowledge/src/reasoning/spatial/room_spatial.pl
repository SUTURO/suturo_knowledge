:- module(room_spatial,
          [ check_inside_room(r,r)
          ]).

check_inside_room(Object,Room) :-
    object_shape_workaround(Room, Frame, ShapeTerm, _, _),
    ShapeTerm = box(DX,DY,_),
    kb_call(is_at(Object, [Frame, [X,Y,_], _])),
    % Assuming z is up direction,
    % X and Y have to be inside the area.
    % so between center + diameter / 2 and center - diameter / 2.
    % center is at 0.
    X =< + DX/2,
    X >= - DX/2,
    Y =< + DY/2,
    Y >= - DY/2.
