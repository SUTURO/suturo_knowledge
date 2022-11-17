:- begin_tests(doors).


:- use_module(library('model/RDFS'), [ has_type/2 ]).
:- use_module(library('lang/terms/triple'), [ triple/3 ]).

:- use_module(library(test)).
:- use_module('doors.pl').



test(setup) :-
    setup_suturo_rooms_test_env,
    setup_suturo_test_rooms.


test(init_doors) :-
    init_doors,
    findall(Door, has_type(Door, hsr_rooms:'Door'), Doors),
    findall(DoorName, (
        member(Door, Doors),
        triple(Door, urdf:'hasURDFName', DoorName)
    ), 
    DoorNames),
    ExpDoorNames = ['door_hall_living-room_1_door_center', 'door_hall_sleeping-room_1_door_center', 'door_hall_kitchen_1_door_center', 'door_living-room_sleeping-room_1_door_center'],
    length(Doors, DoorCount),
    assert_true(DoorCount == 4),
    assert_true(subset(ExpDoorNames, DoorNames)),
    assert_true(subset(DoorNames, ExpDoorNames)).


test(init_door_paths) :-
    init_door_paths,
    findall(Path, has_type(Path, hsr_rooms:'Path'), Paths),
    length(Paths, Count),
    assert_equals(Count, 10),
    forall(member(Path, Paths),
        triple(Path, hsr_rooms:'hasCosts', _)
    ).


test(shortest_path_between_rooms) :-
    has_type(OriginRoom, hsr_rooms:'Kitchen'),
    has_type(DestinationRoom, hsr_rooms:'SleepingRoom'),
    has_type(InBetweenRoom, hsr_rooms:'Hall'),
    shortest_path_between_rooms(OriginRoom, DestinationRoom, Path),

    triple(OriginLocation, soma:'isLinkedTo', OriginRoom),
    triple(OriginLocation, soma:'isLinkedTo', InBetweenRoom),
    triple(DestinationLocation, soma:'isLinkedTo', DestinationRoom),
    triple(DestinationLocation, soma:'isLinkedTo', InBetweenRoom),
    triple(FirstDoor, dul:'hasLocation', OriginLocation),
    triple(SecondDoor, dul:'hasLocation', DestinationLocation),
    ExpectedPath = [FirstDoor, SecondDoor],
    assert_true(subset(ExpectedPath, Path)),
    assert_true(subset(Path, ExpectedPath)).


test(perceiving_pose_of_door) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    perceiving_pose_of_door(Door, [[TX, TY, TZ], [RX, RY, RZ, RW]]),
    assert_true(TX > -4.21), assert_true(TX < -4.19),
    assert_true(TY > -2.01), assert_true(TY < -1.99),
    assert_equals(TZ, 0.0),
    assert_equals(RX, 0.0),
    assert_equals(RY, 0.0),
    assert_true(RZ > -0.71), assert_true(RZ < -0.69),
    assert_true(RW < 0.71), assert_true(RW > 0.69).


test(manipulating_pose_of_door) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    manipulating_pose_of_door(Door, [[TX, TY, TZ], [RX, RY, RZ, RW]]),
    assert_true(TX > -4.14), assert_true(TX < -4.13),
    assert_true(TY > -1.6), assert_true(TY < -1.59),
    assert_equals(TZ, 0.0),
    assert_equals(RX, 0.0),
    assert_equals(RY, 0.0),
    assert_true(RZ > -0.71), assert_true(RZ < -0.69),
    assert_true(RW < 0.71), assert_true(RW > 0.69).


test(passing_pose_of_door) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    passing_pose_of_door(Door, [[TX, TY, TZ], [RX, RY, RZ, RW]]),
    assert_true(TX > -2.6), assert_true(TX < -2.59),
    assert_true(TY > -2.01), assert_true(TY < -1.99),
    assert_equals(TZ, 0.0),
    assert_equals(RX, 0.0),
    assert_equals(RY, 0.0),
    assert_true(RZ > -0.71), assert_true(RZ < -0.69),
    assert_true(RW < 0.71), assert_true(RW > 0.69).


test(get_door_state_when_door_closed) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    get_door_state(Door, State),
    assert_equals(State, 0).


test(get_all_door_states) :-
    get_all_door_states(States),
    findall([Door, 0], has_type(Door, hsr_rooms:'Door'), ExpectedStates),
    assert_true(subset(States, ExpectedStates)),
    assert_true(subset(ExpectedStates, States)).


test(get_angle_to_open_door_when_door_is_closed) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    get_angle_to_open_door(Door, Angle),
    assert_equals(Angle, 1.5708).


test(update_door_state_dynamic) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    get_door_state(Door, BeginState),
    assert_equals(BeginState, 0),
    Angle is pi/2,
    update_door_state_dynamic(Door, Angle),
    get_door_state(Door, EndState),
    assert_equals(EndState, 1).


test(get_door_state_when_door_open) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    get_door_state(Door, State),
    assert_equals(State, 1).


test(get_angle_to_open_door_when_door_is_open) :-
    triple(Door, urdf:'hasURDFName', 'door_hall_kitchen_1_door_center'),
    get_angle_to_open_door(Door, Angle),
    assert_true(Angle > -0.0001),
    assert_true(Angle < 0.0001).

:- end_tests(doors).