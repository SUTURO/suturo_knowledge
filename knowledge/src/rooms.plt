:- begin_tests(rooms).

:- use_module(library('model/RDFS'), [ has_type/2 ]).


:- use_module(library(test)).
:- use_module('rooms.pl').


test(setup) :-
	setup_suturo_rooms_test_env,
	setup_suturo_test_rooms.



test(robot_in_room) :-
	in_room(Room),
	has_type(ExpRoom, hsr_rooms:'Hall'),
	assert_equals(Room, ExpRoom).


test(all_rooms) :-
	findall(Room, has_type(Room, hsr_rooms:'Room'), ExpRooms),
	all_rooms(ActRooms),
	assert_true(subset(ExpRooms, ActRooms)),
	assert_true(subset(ActRooms, ExpRooms)).


:- end_tests(rooms).