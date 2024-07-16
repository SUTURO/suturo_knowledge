:- module(room_creation,
          [ create_room(r, +, +, +, -),
            create_room_entry(r,+,-),
            create_room_exit(r,+,-),
            create_room_entry_exit(r,+,-),
            init_rooms/0
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
    from_current_scope(Scope),
    tf_set_pose(Entry, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope).

create_room_exit(Room, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Exit) :-
    kb_project((is_type(Exit,soma:'Location'),
                is_exit_from(Exit, Room))),
    from_current_scope(Scope),
    tf_set_pose(Exit, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope).

create_room_entry_exit(Room, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Entry) :-
    kb_project((is_type(Entry,soma:'Location'),
                is_exit_from(Entry, Room),
                is_entry_to(Entry, Room))),
    from_current_scope(Scope),
    tf_set_pose(Entry, [Frame, [X,Y,Z], [RX,RY,RZ,RW]], Scope).

% see furniture_creation:init_furnitures for reference
init_rooms :-
    get_urdf_id(URDF),
    urdf_link_names(URDF, Links),
    forall((member(UrdfLink, Links),
            room_type(UrdfLink, Type)
           ),
           init_room(UrdfLink, Type)),
    ros_info('Semantic map rooms initialized').

init_room(UrdfLink, Type) :-
    (  init_room_0(UrdfLink, Type)
    -> ros_info('Loaded room ~w', [UrdfLink])
    ;  ros_warn('Failed to load room ~w', [UrdfLink])).

init_room_0(UrdfLink, RoomType) :-
    furniture_creation:furniture_pose(UrdfLink, Pose),
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF, UrdfLink, box(DX,DY,_), _),
    create_room(RoomType, Pose, DX, DY, Room),
    kb_project(has_urdf_name(Room, UrdfLink)),
    init_connections(URDF,UrdfLink,Room).

room_type(UrdfLink, Type) :-
    atomic_list_concat([_,MapType,room_center_link], ':', UrdfLink),
    room_map_type(MapType, Type).

:- rdf_meta(room_map_type(-,r)).

room_map_type(kitchen, soma:'Kitchen') :- !.
room_map_type(living_room, suturo:'LivingRoom') :- !.
room_map_type(bedroom, suturo:'Bedroom') :- !.
room_map_type(study, suturo:'StudyRoom') :- !.
room_map_type(arena, suturo:'Arena') :- !.
room_map_type(dining_room, suturo:'DiningRoom') :- !.
room_map_type(balcony, suturo:'Balcony') :- !.
room_map_type(bathroom, suturo:'Bathroom') :- !.
room_map_type(lobby, suturo:'Lobby') :- !.
room_map_type(office, suturo:'Office') :- !.
room_map_type(pantry, suturo:'Pantry') :- !.
room_map_type(hallway, suturo:'Hallway') :- !.
%% TODO: add dynamic room type support

init_connections(URDF,UrdfLink,Room) :-
    urdf_link_child_joints(URDF,UrdfLink,Joints),
    forall((member(Joint,Joints),
            urdf_joint_child_link(URDF,Joint,Child)),
           log_result(init_connection_link(Room,Child), 'Created room connection ~w', [Child])).

init_connection_link(Room,UrdfLink) :-
    furniture_creation:furniture_pose(UrdfLink, Pose),
    atomic_list_concat([_,MapType,_], ':', UrdfLink),
    init_connection_link0(MapType, Room, Pose, Location),
    kb_project(has_urdf_name(Location, UrdfLink)).

init_connection_link0(entry, Room, Pose, Location) :-
    create_room_entry(Room, Pose, Location).
init_connection_link0(exit, Room, Pose, Location) :-
    create_room_exit(Room, Pose, Location).
init_connection_link0(entry_exit, Room, Pose, Location) :-
    create_room_entry_exit(Room, Pose, Location).
