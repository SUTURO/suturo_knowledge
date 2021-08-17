:- module(next_object,
    [
        next_object/2,
        create_object_paths/0,
        create_object_path/3,
        assign_object_path_costs/3,
        object_goal_location/2,
        object_costs/3,
        path_costs/3,
        has_path_costs/2,
        has_origin/2,
        has_destination/2
    ]).


next_object(Object, 0) :-
    objects_not_handeled(NotHandledObjects),
    findall(NotHandledObject, 
    (
        member(NotHandledObject, NotHandledObjects),
        is_misplaced(NotHandledObject) 
    ), 
    PossibleObjects),
    tf_lookup_transform('base_footprint', 'map', pose(RobotPosition, _)),
    objects_costs(RobotPosition, PossibleObjects, Costs),
    objects_benefits(PossibleObjects, Benefits),
    max_member([_, NormalizationConstant], Costs),
    findall([Object, CBRatio],
    (
        member([Object, BenefitAtom], Benefits),
        atom_number(BenefitAtom, Benefit),
        member([Object, Cost], Costs),
        CBRatio is Benefit / (Cost / NormalizationConstant)
    ),
    ObjectCBRatios),
    max_member([Object, _], ObjectCBRatios),
    !.


next_object(Object, 1) :-
    create_object_paths,
    create_start_mark_paths,
    ( current_object(CurrentObject)
    -> (
        cheapest_insertion,
        triple(Object, dul:'follows', CurrentObject)
    )
    ; (
        next_object(CheapestObject, 0),
        tell(has_type(StartPosition, hsr_rooms:'StartMark')),
        tf_lookup_transform(base_footprint, 'map', pose(RobotPosition, RobotRotation)),
        object_tf_frame(StartPosition, Frame),
        tell(is_at(Frame, ['map', RobotPosition, RobotRotation])),
        tell(triple(CheapestObject, dul:'follows', StartPosition)),
        object_costs(RobotPosition, CheapestObject, Costs),
        tell(has_type(Path, hsr_rooms:'Path')),
        tell(triple(StartPosition, hsr_rooms:'isOriginOf', Path)),
        tell(triple(CheapestObject, hsr_rooms:'isDestinationOf', Path)),
        tell(triple(Path, hsr_rooms:'hasCosts', Costs)),
        create_start_mark_paths,
        cheapest_insertion,
        triple(Object, dul:'follows', StartPosition)
    )).


current_tour(Tour) :-
    objects_not_handeled(NotHandeledObjects),
    findall([Predecessor, Object],
    (
        member(Object, NotHandeledObjects),
        triple(Object, dul:'follows', Predecessor)
    ),
    Tour).


objects_not_in_current_tour(Objects) :-
    objects_not_handeled(NotHandeledObjects),
    findall(Object,
    (
        member(Object, NotHandeledObjects),
        not triple(Object, dul:'follows', _)
    ),
    Objects).


cheapest_insertion :-
    objects_not_in_current_tour([]).


cheapest_insertion :-
    objects_not_in_current_tour(ObjectsToInsert),
    current_tour(CurrentTour),
    findall([Object, SubTour, Costs],
    (
        member([Object1, Object2], CurrentTour),
        member(Object, ObjectsToInsert),
        object_insertion_costs(Object, Object1, Object2, Costs),
        SubTour = [Object1, Object2]
    ),
    InsertionCosts),
    max_member([_, _, NormalizationConstant], InsertionCosts),
    findall([Object, SubTour, CBRatio],
    (
        member([Object, SubTour, Costs], InsertionCosts),
        object_benefit(Object, BenefitAtom),
        atom_number(BenefitAtom, Benefit),
        CBRatio is Benefit / (Costs / NormalizationConstant)
    ), 
    CBRatios),
    max_member([Object, [Object1, Object2], _], CBRatios),
    insert_object_into_tour(Object, Object1, Object2),
    cheapest_insertion.


object_insertion_costs(ObjectToInsert, Object1, Object2, Costs) :-
    path_costs(Object1, Object2, CurrentPathCosts),
    path_costs(Object1, ObjectToInsert, CostsToNewObject),
    path_costs(ObjectToInsert, Object2, CostsFromNewObject),
    Costs is CostsToNewObject + CostsFromNewObject - CurrentPathCosts.
    

insert_object_into_tour(ObjectToInsert, Object1, Object2) :-
    tripledb_forget(Object2, dul:'follows', Object1),
    tell(triple(ObjectToInsert, dul:'follows', Object1)),
    tell(triple(Object2, dul:'follows', ObjectToInsert)).



current_object(Object) :-
    is_suturo_object(Object),
    handeled(Object),
    ask(triple(SuccessorObject, dul:'follows', Object)),
    not handeled(SuccessorObject).


objects_benefits(Objects, ObjectBenefits) :-
    findall([Object, Benefit],
    (
        member(Object, Objects),
        object_benefit(Object, Benefit)
    ),
    ObjectBenefits).


object_benefit(Object, Benefit) :-
    ask(triple(Object, hsr_objects:'hasConfidenceClassValue', Benefit)).


objects_costs(OriginPosition, Objects, ObjectCosts) :-
    findall([Object, Costs],
    (
        member(Object, Objects),
        object_costs(OriginPosition, Object, Costs)
    ),
    ObjectCosts).


object_costs(OriginPosition, Object, Costs) :-
    distance_to_go(OriginPosition, Object, DistanceToGo),
    robot_velocity(RobotVelocity),
    Costs is DistanceToGo / RobotVelocity.


distance_to_go(OriginPosition, Object, Distance) :-
    distance_to_object(OriginPosition, Object, DistanceToObject),
    distance_to_goal_location(Object, DistanceToLocation),
    Distance is DistanceToObject + DistanceToLocation.


distance_to_object(OriginPosition, Object, Distance) :-
    object_pose(Object, [_, _,ObjectPosition, _]),
    euclidean_distance(OriginPosition, ObjectPosition, Distance).


distance_to_goal_location(Object, Distance) :-
    object_goal_location(Object, GoalPosition),
    object_pose(Object, [_, _,ObjectPosition, _]),
    euclidean_distance(ObjectPosition, GoalPosition, Distance).


object_goal_location(Object, GoalPosition) :-
    object_pose(Object, [_, _,ObjectPosition, _]),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    once(surface_at_predefined_location(GoalSurface, RoomType, FurnitureType)),
    surface_pose_in_map(GoalSurface, [GoalPosition, _]).


create_object_paths :-
    hsr_existing_objects(Objects),
    forall(
    (
        member(Object1, Objects),
        member(Object2, Objects),
        not same_as(Object1, Object2),
        not has_object_path(Object1, Object2),
        not handeled(Object2),
        is_misplaced(Object2)
    ),
    (
        create_object_path(Object1, Object2, Path1),
        create_object_path(Object2, Object1, Path2),
        assign_object_path_costs(Object1, Object2, Path1),
        assign_object_path_costs(Object2, Object1, Path2)
    )).

create_start_mark_paths :-
    objects_not_handeled(NotHandeledObjects),
    forall(
    (
        has_type(StartMark, hsr_rooms:'StartMark'),
        member(Object, NotHandeledObjects),
        is_misplaced(Object),
        not has_object_path(StartMark, Object)
    ),
    (
        create_object_path(StartMark, Object, Path),
        object_tf_frame(StartMark, Frame),
        tf_lookup_transform(Frame, 'map', pose(StartPosition, _)),
        object_costs(StartPosition, Object, Costs),
        tell(triple(Path, hsr_rooms:'hasCosts', Costs))
    )).


create_object_path(Object1, Object2, Path) :-
    tell(has_type(Path, hsr_rooms:'Path')),
    tell(triple(Object1, hsr_rooms:'isOriginOf', Path)),
    tell(triple(Object2, hsr_rooms:'isDestinationOf', Path)).


has_object_path(Object1, Object2) :-
    ( triple(Object1, hsr_rooms:'isOriginOf', Path), 
      triple(Object2, hsr_rooms:'isDestinationOf', Path));
    ( triple(Object2, hsr_rooms:'isOriginOf', Path), 
      triple(Object1, hsr_rooms:'isDestinationOf', Path)).


assign_object_path_costs(Origin, Destination, Path) :-
    object_goal_location(Origin, GoalPosition),
    object_costs(GoalPosition, Destination, Costs),
    tell(triple(Path, hsr_rooms:'hasCosts', Costs)).


path_costs(Origin, Destination, Costs) :-
    has_origin(Path, Origin),
    has_destination(Path, Destination),
    has_path_costs(Path, Costs).

has_path_costs(Path, Costs) :-
    triple(Path, hsr_rooms:'hasCosts', Costs).


is_path(Path) :-
    has_type(Path, hsr_rooms:'Path').


has_origin(Path, Origin) :-
    triple(Path, hsr_rooms:'hasOrigin', Origin);
    triple(Origin, hsr_rooms:'isOriginOf', Path).


has_destination(Path, Destination) :-
    triple(Path, hsr_rooms:'hasDestination', Destination);
    triple(Destination, hsr_rooms:'isDestinationOf', Path).








    

