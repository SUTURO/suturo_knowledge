:- module(next_object,
    [
        next_object/2,
        create_object_paths/0,
        object_goal_location/2,
        path_costs_normalization_constant/1,
        object_costs/3
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





current_object(Object) :-
    is_suturo_object(Object),
    handeled(Object),
    ask(triple(Object, hsr_objects:'hasSuccessorObject', SuccessorObject)),
    not handeled(SuccessorObject).
    



all_objects_cost_benefit_ratios(OriginPosition, PossibleObjects, CBRatios) :-
    all_objects_normalized_costs_and_benefits(OriginPosition, NormalizedCostsAndBenefits),
    findall([Object, CBRatio],
    (
        member([Object, Costs, Benefit], PossibleObjects, NormalizedCostsAndBenefits),
        CBRatio is Costs / Benefit
    ),
    CBRatios).


all_objects_normalized_costs_and_benefits(OriginPosition, PossibleObjects, NormalizedCostsAndBenefits) :-
    all_objects_costs_and_benefits(OriginPosition, PossibleObjects, CostsAndBenefits),
    max_member([_, NormalizationConstant, _], CostsAndBenefits),
    findall([Object, NormalizedCosts, Benefit],
    (
        member([Object, Costs, Benefit], CostsAndBenefits),
        NormalizedCosts is Costs / NormalizationConstant
    ),
    NormalizedCostsAndBenefits).


all_objects_costs_and_benefits(OriginPosition, PossibleObjects, CostsAndBenefits) :-
    findall([Object, Costs, Benefit],
    (
        member(Object, PossibleObjects),
        object_costs(OriginPosition, Object, Costs),
        object_benefit(Object, Benefit)
    ),
    CostsAndBenefits).


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
        member(Object1, Objects), member(Object2, Objects),
        not same_as(Object1, Object2),
        not has_object_path(Object1, Object2)
    ),
    (
        create_object_path(Object1, Object2, Path1),
        create_object_path(Object2, Object1, Path2),
        assign_object_path_costs(Object1, Object2, Path1),
        assign_object_path_costs(Object2, Object1, Path2)
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


has_path_costs(Path, Costs) :-
    triple(Path, hsr_rooms:'hasCosts', Costs).


is_path(Path) :-
    has_type(Path, hsr_rooms:'Path').


has_origin(Path, Origin) :-
    triple(Path, hsr_rooms:'hasOrigin', Origin).


has_destination(Path, Destination) :-
    triple(Path, hsr_rooms:'hasDestination', Destination).


path_costs_normalization_constant(NormalizationConstant) :-
    tf_lookup_transform('map', 'base_footprint', pose(RobotPosition, _)),
    objects_not_handeled(NotHandledObjects),
    findall(Costs,
    (
        is_path(Path),
        has_origin(Path, Origin), 
        member(Origin, NotHandledObjects),
        has_destination(Path, Destination), 
        member(Destination, NotHandledObjects),
        has_path_costs(Path, Costs)
    ), 
    ObjectToObjectCosts),
    findall(Costs,
    (
        member(Destination, NotHandledObjects),
        object_costs(RobotPosition, Destination, Costs)
    ),
    RobotToObjectCosts),
    append(ObjectToObjectCosts, RobotToObjectCosts, Costs),
    max_member(NormalizationConstant, Costs).


normalization_constant(Costs, NormalizationConstant) :-
    max_member(NormalizationConstant, Costs).







    

