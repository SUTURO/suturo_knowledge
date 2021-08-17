:- module(evaluate_next_object,
    [
        evaluate_next_object/5,
        init_robot_start_position/0,
        generate_random_objects/1
    ]).


evaluate_next_object(Algorithm, ObjectCount, Time, RemainingTime, FinalScore) :-
    init_robot_start_position,
    generate_random_objects(ObjectCount),
    perform_next_object_search(Algorithm, Time, 0.0, RemainingTime, FinalScore).


perform_next_object_search(Algorithm, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    Time > 0.0,
    writeln("Next Iteration Perform Next Object Search"),
    ( next_object(Object, Algorithm)
    -> (writeln("Next chosen object"),
        writeln(Object),
        once(object_goal_location(Object, GoalPosition)),
        move_robot(GoalPosition),
        move_next_object(Object, GoalPosition, Costs, ObjectScore),
        NewTime is Time - Costs,
        ( not NewTime < 0.0
        -> NewScore is CurrentScore + ObjectScore
        ; NewScore is CurrentScore)
    );
    (
        next_surface(GoalPosition, Costs),
        move_robot(GoalPosition),
        NewTime is Time - Costs,
        NewScore is CurrentScore
    )),
    writeln("Time and Score after one cycle"),
    writeln(NewTime),
    writeln(NewScore),
    perform_next_object_search(Algorithm, NewTime, NewScore, UpdatedTime, UpdatedScore),
    !.


perform_next_object_search(_, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    (Time < 0.0; Time == 0.0),
    UpdatedScore is CurrentScore,
    UpdatedTime is Time,
    !.

perform_next_object_search(_, Time, CurrentScore, UpdatedTime, UpdatedScore) :-
    surfaces_not_visited(NotVisitedSurfaces),
    forall(member(Surface, NotVisitedSurfaces), has_type(Surface, hsr_rooms:'Floor')),
    objects_not_handeled(NotHandledObjects),
    forall(member(Object, NotHandledObjects), not is_misplaced(Object)),
    UpdatedScore is CurrentScore,
    UpdatedTime is Time.

next_surface(SurfacePosition, Costs) :-
    surfaces_not_visited(Surfaces),
    findall(Surface, (member(Surface, Surfaces), not has_type(Surface, hsr_rooms:'Floor')), PossibleSurfaces),
    predsort(compareDistances, PossibleSurfaces, SortedSurfaces),
    nth0(0, SortedSurfaces, Surface),
    surface_pose_in_map(Surface, [SurfacePosition, _]),
    has_surface(Furniture, Surface),
    furniture_surfaces(Furniture, FurnitureSurfaces),
    forall(member(FurnitureSurface, FurnitureSurfaces), set_surface_visited(FurnitureSurface)),
    objects_on_furniture(Furniture, ObjectsOnFurniture),
    forall(member(Object, ObjectsOnFurniture), set_object_not_handeled(Object)),
    tf_lookup_transform(base_footprint, 'map', pose(RobotPosition, _)),
    euclidean_distance(RobotPosition, SurfacePosition, DistanceToGo),
    robot_velocity(RobotVelocity),
    Costs is DistanceToGo / RobotVelocity.


move_next_object(Object, GoalPosition, Costs, Score) :-
    object_tf_frame(Object, ObjectFrame),
    forget_object_at_location(Object),
    tell(is_at(ObjectFrame, ['map', GoalPosition, [0, 0, 0, 1]])),
    tf_lookup_transform(base_footprint, 'map', pose(RobotPosition, _)),
    object_costs(RobotPosition, Object, Costs),
    set_object_handeled(Object),
    object_on_furniture(Object, Furniture),
    furniture_surfaces(Furniture, FurnitureSurfaces),
    forall(member(FurnitureSurface, FurnitureSurfaces), set_surface_visited(FurnitureSurface)),
    objects_on_furniture(Furniture, ObjectsOnFurniture),
    forall((member(OtherObject, ObjectsOnFurniture), not same_as(OtherObject, Object)), set_object_not_handeled(OtherObject)),
    generate_score(Object, Score).


move_robot(GoalPosition) :-
    tell(is_at(base_footprint, ['map', GoalPosition, [0, 0, 0, 1]])).


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
    set_random(seed(111)),
    foreach(between(1, Count, _), generate_random_object).


generate_random_object :-
    random_object_classes(Classes),
    random_member(Class, Classes),
    random_object_locations(Class, Surfaces),
    random_member(Surface, Surfaces),
    surface_pose_in_map(Surface, [[X, Y, Z], _]),
    random(0.4, 1.0, ClassConfidence),
    create_object(Class, ClassConfidence, ['map', [X, Y, Z], [0.0, 0.0, 0.0, 1.0]], [0.1, 0.1, 0.1], _, 1.0, [255, 0, 0], 1.0, ObjId).
    %set_object_handeled(ObjId).


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
    findall(Furniture,
    (
        member(Furniture, AllFurnitures),
        has_type(Furniture, FurnitureType),
        not
        ( triple(Class, hsr_rooms:'hasPredefinedLocation', PredefinedLocation),
          triple(PredefinedLocation, hsr_rooms:'isOntopOf', FurnitureType)
        )
    ),
    Furnitures),
    findall(Surface, 
    (
        member(Furniture, Furnitures),
        has_surface(Furniture, Surface)
    ), 
    Surfaces).