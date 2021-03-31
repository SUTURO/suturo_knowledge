:- module(test,
    [
        setup_suturo_test_env/0,
        get_suturo_test_env/1,
        setup_suturo_test_surfaces/0,
        setup_suturo_test_objects/0
    ]
).


:- use_module(library('ros/urdf/URDF')).
:- use_module(library('object_state'), [create_object]).


setup_suturo_test_env :-
    get_suturo_test_env(URDF),
    ros_package_path('knowledge', X),
    atom_concat(X, '/urdf/gz_sim_v1_openable_door.urdf', Filepath),
    urdf_load_file(URDF, Filepath).


get_suturo_test_env(URDF) :-
    URDF = testenv.


setup_suturo_test_surfaces :-
    get_suturo_test_env(URDF),
    urdf_link_names(URDF, Links),
    tell(triple(ground,hsr_objects:'isSurfaceType',ground)),
    forall(member(Link, Links),
    (
        ( sub_string(Link,_,_,_,shelf)
        ->tell(triple(Link,hsr_objects:'isSurfaceType',shelf))
        ;
        ( sub_string(Link,_,_,_,table)
        ->tell(triple(Link,hsr_objects:'isSurfaceType',table))
        ;
        ( sub_string(Link,_,_,_,bucket)
        ->tell(triple(Link,hsr_objects:'isSurfaceType',bucket))
        ;tell(triple(Link,hsr_objects:'isSurfaceType',other)))
    )))).


setup_suturo_test_objects(Objects) :-
    % create two objects on table
    create_object("http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Bowl", 0.8, ['map', [0.4, 0.75, 0.3099365], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, Obj1Id),
    create_object("http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan", 0.9, ['map', [0.3, 0.8, 0.3099365], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, Obj2Id),
    create_object("http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan", 0.75, ['map', [0.0, 0.0, 0.81], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, Obj3Id),
    create_object("http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Spoon", 0.96, ['map', [0.0, 0.0, 0.44], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [0, 0, 255], 1.0, Obj4Id),
    Objects = [Obj1Id, Obj2Id, Obj3Id, Obj4Id].
    