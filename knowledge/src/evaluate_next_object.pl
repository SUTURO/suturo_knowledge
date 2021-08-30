:- module(evaluate_next_object,
    [
        evaluate_next_object/5,
        init_robot_start_position/0,
        generate_random_objects/1
    ]).


evaluate_next_object(Algorithm, ObjectCount, Time, RemainingTime, FinalScore) :-
    set_random(seed(139)),
    init_robot_start_position,
    generate_random_objects(ObjectCount),
    hsr_existing_objects(AllObjects),
    findall([Object, Score], 
    (
        member(Object, AllObjects),
        generate_score(Object, Score)
    ),
    ObjectScores),
    writeln("Object Scores"),
    writeln(ObjectScores),
    perform_next_object_search(Algorithm, ObjectScores, Time, 0.0, RemainingTime, FinalScore).


perform_next_object_search(Algorithm, ObjectScores, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    Time > 0.0,
    writeln("Next Iteration Perform Next Object Search"),
    ( next_object(Object, Algorithm)
    -> (writeln("Next chosen object"),
        writeln(Object),
        once(object_goal_location(Object, GoalPosition)),
        surface_pose_in_map(GoalSurface, [GoalPosition, _]),
        has_surface(GoalFurniture, GoalSurface),
        writeln("Move object to Furniture"),
        writeln(GoalFurniture),
        move_next_object(Object, GoalPosition, Costs),
        move_robot(GoalPosition),
        member([Object, ObjectScore], ObjectScores),
        NewTime is Time - Costs,
        ( not NewTime < 0.0
        -> NewScore is CurrentScore + ObjectScore
        ; NewScore is CurrentScore)
    );
    (
        next_surface(Surface, GoalPosition, Costs),
        writeln("Next chosen Surface"),
        writeln(Surface),
        move_robot(GoalPosition),
        NewTime is Time - Costs,
        NewScore is CurrentScore
    )),
    writeln("Time and Score after one cycle"),
    writeln(NewTime),
    writeln(NewScore),
    perform_next_object_search(Algorithm, ObjectScores, NewTime, NewScore, UpdatedTime, UpdatedScore),
    !.


perform_next_object_search(_, _, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    (Time < 0.0; Time == 0.0),
    UpdatedScore is CurrentScore,
    UpdatedTime is Time,
    !.

perform_next_object_search(_, _, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    surfaces_not_visited(NotVisitedSurfaces),
    forall(member(Surface, NotVisitedSurfaces), has_type(Surface, hsr_rooms:'Floor')),
    objects_not_handeled(NotHandledObjects),
    forall(member(Object, NotHandledObjects), not is_misplaced(Object)),
    UpdatedScore is CurrentScore,
    UpdatedTime is Time.

next_surface(Surface, SurfacePosition, Costs) :-
    surfaces_not_visited(Surfaces),
    findall(Surface, (member(Surface, Surfaces), not has_type(Surface, hsr_rooms:'Floor')), PossibleSurfaces),
    predsort(compareDistances, PossibleSurfaces, SortedSurfaces),
    nth0(0, SortedSurfaces, Surface),
    surface_pose_in_map(Surface, [[X, Y, Z], _]),
    has_surface(Furniture, Surface),
    furniture_surfaces(Furniture, FurnitureSurfaces),
    forall(member(FurnitureSurface, FurnitureSurfaces), set_surface_visited(FurnitureSurface)),
    objects_on_furniture(Furniture, ObjectsOnFurniture),
    forall((member(Object, ObjectsOnFurniture), is_misplaced(Object)), set_object_not_handeled(Object)),
    tf_lookup_transform(base_footprint, 'map', pose(RobotPosition, _)),
    euclidean_distance(RobotPosition, [X, Y, 0], DistanceToGo),
    robot_velocity(RobotVelocity),
    Costs is DistanceToGo / RobotVelocity,
    SurfacePosition = [X, Y, Z].


move_next_object(Object, [X, Y, Z], Costs) :-
    is_at(base_footprint, ['map', RobotPosition, _]),
    object_costs(RobotPosition, Object, PathCosts),
    robot_velocity(RobotVelocity),
    Costs is PathCosts / RobotVelocity,
    forget_object_at_location(Object),
    object_tf_frame(Object, ObjectFrame),
    object_dimensions(Object, _, _, ObjectHeight),
    ZOffset is Z + ObjectHeight/2,
    tell(is_at(ObjectFrame, ['map', [X, Y, ZOffset], [0, 0, 0, 1]])),
    set_object_handeled(Object),
    object_on_furniture(Object, Furniture),
    furniture_surfaces(Furniture, FurnitureSurfaces),
    forall(member(FurnitureSurface, FurnitureSurfaces), set_surface_visited(FurnitureSurface)),
    objects_on_furniture(Furniture, ObjectsOnFurniture),
    forall((member(OtherObject, ObjectsOnFurniture), not same_as(OtherObject, Object), is_misplaced(OtherObject)), set_object_not_handeled(OtherObject)).


move_robot([X, Y, _]) :-
    tell(is_at(base_footprint, ['map', [X, Y, 0], [0, 0, 0, 1]])).


init_robot_start_position :-
    tell(is_individual(base_footprint)),
    tell(is_at(base_footprint, ['map', [0, 1.5, 0], [0, 0, 0, 1]])).


generate_score(Object, Score) :-
    triple(Object, hsr_objects:'hasConfidenceClassValue', ConfidenceAtom),
    atom_number(ConfidenceAtom, Confidence),
    random(0, 1.0, Random),
    ( Random > Confidence
    -> Score is 0.0
    ; Score is 1.0
    ).


generate_random_objects(Count) :-
    foreach(between(1, Count, _), generate_random_object).


generate_random_object :-
    random_object_classes(Classes),
    random_member(Class, Classes),
    random_object_locations(Class, Surfaces),
    random_member(Surface, Surfaces),
    surface_pose_in_map(Surface, [[X, Y, Z], _]),
    random(0.4, 1.0, ClassConfidence),
    create_object(Class, ClassConfidence, ['map', [X, Y, Z], [0.0, 0.0, 0.0, 1.0]], [0.1, 0.1, 0.1], _, 1.0, [255, 0, 0], 1.0, ObjId),
    set_object_handeled(ObjId).


random_object_classes(Classes) :-
    Classes = [
        'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Wasserkanne',
        'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Coffee', 
        'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Fruit', 
        'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#Chips', 
        'http://www.ease-crc.org/ont/SOMA.owl#Fork', 
        'http://www.ease-crc.org/ont/SOMA.owl#DishwasherTab'
    ].


random_object_locations(Class, Surfaces) :-
    all_furnitures(AllFurnitures),
    findall(NotPossibleFurniture,
    (
        member(NotPossibleFurniture, AllFurnitures),
        has_type(NotPossibleFurniture, FurnitureType), 
        transitive(subclass_of(Class, SuperClass)),
        triple(SuperClass, hsr_rooms:'hasPredefinedLocation', Location),
        triple(Location, knowrob:'isOntopOf', FurnitureType)
    ),
    NotPossibleFurnitures),
    writeln(NotPossibleFurnitures),
    findall(Furniture,
    (
        member(Furniture, AllFurnitures),
        not member(Furniture, NotPossibleFurnitures)
    ), Furnitures),
    findall(Surface, 
    (
        member(Furniture, Furnitures),
        has_surface(Furniture, Surface)
    ), 
    Surfaces).