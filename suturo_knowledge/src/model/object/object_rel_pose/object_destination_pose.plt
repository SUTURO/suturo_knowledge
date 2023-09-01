:- use_module(library('rostest')).

:- use_module(library('util/suturo_test')).

:- use_module(library('model/object/object_creation'), [create_object/4]).

:- use_module('object_destination_pose').

:- begin_rdf_tests(
       'object_destination_pose',
       'package://suturo_knowledge/owl/suturo.owl',
       [ namespace('http://www.ease-crc.org/ont/SUTURO-test.owl#')
       ]).

test(for_loop1, all(X == [-3,-2,-1,0,1,2,3])) :-
    object_destination_pose:for_loop(-3, 3, 1, X).

test(possible_pose1,
     [ setup(test_setup),
       cleanup(test_cleanup)]) :-
    create_object(Table, test:'Table', [map, [0,0,1],[0,0,0,1]], [shape(box(3,2,1))]),
    create_object(Obj, test:'Obj', [map, [0,-1,0], [0,0,0,1]]),
    forall(object_destination_pose:possible_pose(Table, Obj, 0.2, [_Frame, [X,Y,0], [0,0,0,1]]),
           (assert_equals(X,-1.25),
            assert_true((-2 =< Y, Y =< 2)))).

test(best_fitting_destination1,
     [ setup(test_setup),
       cleanup(test_cleanup)]) :-
    create_object(Table1, test:'Table', [map, [-1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    create_object(Table2, test:'Table', [map, [1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    %
    create_object(Obj, test:'Box', [map, [0,0,0], [0,0,0,1]]),
    create_object(Similar, test:'Box', [map, [-1,0,1.5], [0,0,0,1]]),
    create_object(_NotSimilar, test:'Sphere', [map, [1,0,1.5], [0,0,0,1]]),
    kb_project((subclass_of(test:'Box', test:top),
                subclass_of(test:'Sphere', test:top),
                triple(test:'Box', suturo:hasDestinationLocation, Table2),
                triple(test:'Box', suturo:hasDestinationLocation, Table1)
               )),
    once(object_destination_pose:best_fitting_destination(Obj, NextTo, Dest)),
    assert_equals(Dest, Table1),
    assert_equals(NextTo, Similar).

test(object_destination_pose_normal,
     [ setup(test_setup),
       cleanup(test_cleanup)]) :-
    ignore(reset:reset_user_data),
    create_object(Table1, test:'Table', [map, [-1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    kb_project(has_urdf_name(Table1, table_urdf_for_manipulation)),
    %
    create_object(Obj, test:'Box', [map, [0,0,0], [0,0,0,1]]),
    create_object(_Similar, test:'Box', [map, [-1,0,1.5], [0,0,0,1]]),
    kb_project((subclass_of(test:'Box', test:top),
                triple(test:'Box', suturo:hasDestinationLocation, Table1)
               )),
    object_destination_pose:object_destination_pose(Obj, [], [Frame, _Pos, Rot]),
    assert_equals(Frame, 'iai_kitchen/table_urdf_for_manipulation'),
    assert_equals(Rot, [0,0,0,1]).

test(object_destination_pose_empty,
     [ setup(test_setup),
       cleanup(test_cleanup)]) :-
    ignore(reset:reset_user_data),
    create_object(Table1, test:'Table', [map, [-1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    kb_project(has_urdf_name(Table1, table_urdf_for_manipulation)),
    %
    create_object(Obj, test:'Box', [map, [0,0,0], [0,0,0,1]]),
    kb_project((subclass_of(test:'Box', test:top),
                triple(test:'Box', suturo:hasDestinationLocation, Table1)
               )),
    object_destination_pose:object_destination_pose(Obj, [], [Frame, _Pos, Rot]),
    assert_equals(Frame, 'iai_kitchen/table_urdf_for_manipulation'),
    assert_equals(Rot, [0,0,0,1]).

test(object_destination_pose_wrong_category,
     [ setup(test_setup),
       cleanup(test_cleanup)]) :-
    ignore(reset:reset_user_data),
    create_object(Table1, test:'Table', [map, [-1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    kb_project(has_urdf_name(Table1, table1_urdf_for_manipulation)),
    create_object(Table2, test:'Table', [map, [1,0,1],[0,0,0,1]],
                  [shape(box(1,1,1)), data_source(semantic_map)]),
    kb_project(has_urdf_name(Table2, table2_urdf_for_manipulation)),
    %
    create_object(Obj, test:'Box', [map, [0,0,0], [0,0,0,1]]),
    create_object(NotSimilar, test:'Square', [map, [-1,0,1.5], [0,0,0,1]]),
    kb_project((subclass_of(test:'Box', test:'body'),
                subclass_of(test:'body', test:'top'),
                subclass_of(test:'Suqare', test:'shape'),
                subclass_of(test:'shape', test:'top'),
                triple(test:'Box', suturo:hasDestinationLocation, Table1),
                triple(test:'Box', suturo:hasDestinationLocation, Table2)
               )),
    object_destination_pose:object_destination_pose(Obj, [], [Frame, _Pos, Rot]),
    assert_equals(Frame, 'iai_kitchen/table2_urdf_for_manipulation'),
    assert_equals(Rot, [0,0,0,1]).


:- end_rdf_tests('object_destination_pose').
