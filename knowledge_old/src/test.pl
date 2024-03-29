:- module(test,
    [
        setup_suturo_test_env/0,
        setup_suturo_rooms_test_env/0,
        get_suturo_test_env/1,
        setup_suturo_test_surfaces/0,
        setup_suturo_test_rooms/0,
        setup_suturo_test_furnitures/0,
        setup_suturo_test_objects/0,
        get_suturo_test_objects/1,
        cleanup_suturo_test_objects/0,
        setup_suturo_test_source_surfaces/1,
        setup_suturo_test_target_surfaces/1,
        reset_goal_surfaces/0
    ]
).


:- use_module(library('ros/urdf/URDF')).
:- use_module(library('object_state'), [create_object/9]).
:- use_module(library('rooms'), [init_rooms/0]).
:- use_module(library('surfaces'), [init_furnitures/0]).

setup_suturo_test_env :-
    get_suturo_test_env(URDF),
    ros_package_path('knowledge', X),
    atom_concat(X, '/urdf/gz_sim_v1_openable_door.urdf', Filepath),
    urdf_load_file(URDF, Filepath),
    urdf_set_pose_to_origin(URDF,map).


setup_suturo_rooms_test_env :-
    get_suturo_test_env(URDF),
    ros_package_path('knowledge', X),
    atom_concat(X, '/urdf/suturo_rooms_2.urdf', Filepath),
    urdf_load_file(URDF, Filepath),
    urdf_set_pose_to_origin(URDF,map).


get_suturo_test_env(URDF) :-
    URDF = arena.


setup_suturo_test_surfaces :-
    tell(triple('table_center', hsr_objects:'isSurfaceType', table)),
    tell(triple('table_clone_center', hsr_objects:'isSurfaceType', table)),
    tell(triple('bookshelf_floor_0_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bookshelf_floor_1_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bookshelf_floor_2_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bookshelf_clone_floor_0_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bookshelf_clone_floor_1_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bookshelf_clone_floor_2_piece', hsr_objects:'isSurfaceType', shelf)),
    tell(triple('bucket_center', hsr_objects:'isSurfaceType', bucket)),
    tell(triple(ground, hsr_objects:'isSurfaceType', ground)).


cleanup_suturo_test_surfaces :-
    tripledb_forget('table_center', hsr_objects:'isSurfaceType', _),
    tripledb_forget('table_clone_center', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_floor_0_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_floor_1_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_floor_2_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_clone_floor_0_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_clone_floor_1_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bookshelf_clone_floor_2_piece', hsr_objects:'isSurfaceType', _),
    tripledb_forget('bucket_center', hsr_objects:'isSurfaceType', bucket),
    tripledb_forget(ground, hsr_objects:'isSurfaceType', ground).


setup_suturo_test_furnitures :-
    findall(Furniture, has_type(Furniture, soma:'DesignedFurniture'), Furnitures),
    length(Furnitures, Count),
    (Count == 0
    ->(
        init_furnitures
    )).


setup_suturo_test_rooms :-
    findall(Room, has_type(Room, hsr_rooms:'Room'), Rooms),
    length(Rooms, Count),
    (Count == 0
    ->(
        init_rooms
    )).


%setup_suturo_test_objects :-
%    get_suturo_test_objects(TestObjects),
%    length(TestObjects, Count),
%    (Count == 0
%    -> (
%        create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Bowl', 0.8, ['map', [1.3, -0.17, 0.619873], [0, 0, 0, 1]], [0.3, 0.3, 0.3], _, 1.0, [255, 0, 0], 1.0, _),
%        create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan', 0.9, ['map', [1.3, 0.07, 0.619873], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, _),
%        create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan', 0.9, ['map', [0.7, 4.8, 0.81], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, _),
%        create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Spoon', 0.96, ['map', [0.7, 4.8, 0.44], [0, 0, 0, 1]], [0.2, 0.2, 0.2], _, 1.0, [0, 0, 255], 1.0, _)
%    )).  


setup_suturo_test_objects :-
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Bowl', 0.8, ['map', [2.5, 4.0, 0.619873], [0, 0, 0, 1]], [0.3, 0.3, 0.3], _, 1.0, [255, 0, 0], 1.0, _),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan', 0.9, ['map', [1.5, 1.0, 0.619873], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, _),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Cokecan', 0.9, ['map', [-0.4, -3.6, 0.619873], [0, 0, 0, 1]], [0.4, 0.4, 0.4], _, 1.0, [255, 0, 0], 1.0, _),
    create_object('http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Spoon', 0.96, ['map', [-2.6, -3.8, 0.582], [0, 0, 0, 1]], [0.2, 0.2, 0.2], _, 1.0, [0, 0, 255], 1.0, _).   


get_suturo_test_objects(TestObjects) :-
    findall(Object, 
    (
        has_type(Object, dul:'PhysicalObject'), 
        not has_type(Object, soma:'DesignedFurniture'),
        not has_type(Object, soma:'Surface'),
        not has_type(Object, hsr_rooms:'Room'),
        not has_type(Object, hsr_rooms:'Door')
    ),
    TestObjects).
    

cleanup_suturo_test_objects :-
    get_suturo_test_objects(TestObjects),
    forall(member(Obj, TestObjects), tripledb_forget(Obj, _, _)).


setup_suturo_test_source_surfaces(Surfaces) :-
    forall(triple(Source, hsr_objects:'sourceOrTarget', source), tripledb_forget(Source, hsr_objects:'sourceOrTarget', source)),
    forall(member(Surface, Surfaces), tell(triple(Surface, hsr_objects:'sourceOrTarget', source))).

setup_suturo_test_target_surfaces(Surfaces) :-
    forall(triple(Target, hsr_objects:'sourceOrTarget', target), tripledb_forget(Target, hsr_objects:'sourceOrTarget', target)),
    forall(member(Surface, Surfaces), tell(triple(Surface, hsr_objects:'sourceOrTarget', target))).

reset_goal_surfaces :-
    forall(triple(Object, supposedSurface, _), tripledb_forget(Object, supposedSurface, _)),
    forall(triple(Object, context, _), tripledb_forget(Object, context, _)),
    forall(triple(Object, refObject, _), tripledb_forget(Object, refObject, _)).