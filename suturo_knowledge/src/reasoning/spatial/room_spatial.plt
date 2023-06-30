:- use_module(library('rostest')).
:- use_module(library('util/reset'), [reset_user_data/0]).

:- use_module('room_spatial').
:- use_module(library('model/rooms/room_creation'), [create_room/5]).
:- use_module(library('model/object/object_creation'), [create_object/3]).

:- begin_rdf_tests(
       'room_spatial',
       'package://suturo_knowledge/owl/suturo.owl',
       [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#'),
         cleanup(object_creation_cleanup)
       ]).

object_creation_cleanup :-
    ignore(reset_user_data).

test('check_inside_room') :-
    room_types:new_room_type(test:room),
    create_room(test:room, [map, [0,0,0], [0,0,0,1]], 2, 4, Room),
    create_object(Obj, test:obj, [map, [1,2,0], [0,0,0,1]]),
    assert_true(check_inside_room(Obj, Room)),
    % test the integration in create_object
    assert_true(kb_call(is_inside_of(Obj,Room))).

:- end_rdf_tests('room_spatial').
