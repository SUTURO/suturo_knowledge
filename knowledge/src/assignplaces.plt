:- begin_tests('assignplaces').

:- use_module(library('test')).

:- use_module(library('ros/urdf/URDF')).
:- use_module(library('model/SOMA/OBJ')).

:- include(library('assignplaces')).


:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


%test(object_goal_pose_offset_when_target_shelf) :-
    %get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    %setup_suturo_test_source_surfaces(['table_center']),
    %setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    %object_goal_pose_offset_(Cokecan1, [[X, Y, Z], Rotation], Context).


test(object_goal_pose_offset_when_target_bucket) :-
    get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    setup_suturo_test_source_surfaces(['table_center']),
    setup_suturo_test_target_surfaces(['bucket_center']),
    object_goal_pose_offset_(Cokecan1, [[X, Y, Z], Rotation], Context),
    writeln(X),
    writeln(Y),
    writeln(Z).

test(fail) :-
    fail.


:- end_tests('assignplaces').