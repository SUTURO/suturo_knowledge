:- begin_tests('assignplaces').

:- use_module(library('test')).

:- include(library('assignplaces')).


:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


%test(object_goal_pose_offset) :-
    %get_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]),
    %setup_suturo_test_source_surfaces(['table_center']),
    %setup_suturo_test_target_surfaces(['bookshelf_floor_0_piece', 'bookshelf_floor_1_piece', 'bookshelf_floor_2_piece']),
    %object_goal_pose_offset_(Cokecan1, [[X, Y, Z], Rotation], Context),
    %writeln(X),
    %writeln(Y),
    %writeln(Z).


test(fail) :-
    fail.


:- end_tests('assignplaces').