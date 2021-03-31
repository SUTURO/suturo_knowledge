:- use_module(library('db/tripledb_tests')).

:- begin_tripledb_tests(
		'assignplaces',
        'package://knowledge/owl/testing.owl',
       	[ namespace('http://www.semanticweb.org/suturo/ontologies/2021/0/testing#')]
).

:- use_module(library('test')).
:- use_module(library('assignplaces')).


:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects([Bowl1, Cokecan1, Cokecan2, Spoon1]).


test(object_goal_pose_offset) :-
    forall(triple(Surface, hsr_objects:'isSurfaceType', shelf), tell(triple(Surface, hsr_objects:'sourceOrTarget', target))),
    forall(triple(Surface, hsr_objects:'isSurfaceType', table), tell(triple(Surface, hsr_objects:'sourceOrTarget', source))),
    object_goal_pose_offset_(Cokecan1, [[X, Y, Z], Rotation], Context).


test(fail) :-
    fail.


:- end_tripledb_tests('assignplaces').