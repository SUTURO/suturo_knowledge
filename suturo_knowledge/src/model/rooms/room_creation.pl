:- module(room_creation,
          [ create_room(r, +, +, +, -),
            create_room_entry(r,+,-),
            create_room_exit(r,+,-),
            create_room_entry_exit(r,+,-)
          ]).

:- use_module(room_relations).
:- use_module(library('util/util'), [from_current_scope/1]).

%% create_room(+RoomType, +PoseStamped, +Depth, +Width, -IRI) is det.
%
% create a room with the specified type, PoseStamped, Width(y) and Depth(x).
create_room(RoomType, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Depth, Width, IRI) :-
    kb_project(is_type(IRI, RoomType)),
    from_current_scope(Scope),
    tf_set_pose(IRI, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope),
    kb_project((is_type(Shape, soma:'Shape'),
                is_type(SR, soma:'BoxShape'),
                triple(IRI,soma:hasShape,Shape),
                triple(Shape,dul:hasRegion,SR),
                triple(SR, soma:hasDepth,  Depth),
                triple(SR, soma:hasWidth,  Width),
                triple(SR, soma:hasHeight, 4))).

create_room_entry(Room, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Entry) :-
    kb_project((is_type(Entry,soma:'Location'),
                is_entry_to(Entry, Room))),
    tf_set_pose(Entry, [Frame, [X,Y,Z], [RX,RY,RZ,RW]]).

create_room_exit(Room, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Exit) :-
    kb_project((is_type(Exit,soma:'Location'),
                is_exit_from(Exit, Room))),
    tf_set_pose(Exit, [Frame, [X,Y,Z], [RX,RY,RZ,RW]]).

create_room_entry_exit(Room, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Entry) :-
    kb_project((is_type(Entry,soma:'Location'),
                is_exit_from(Entry, Room),
                is_entry_to(Entry, Room))),
    tf_set_pose(Entry, [Frame, [X,Y,Z], [RX,RY,RZ,RW]]).
