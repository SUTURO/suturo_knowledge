:- module(next_object,
    [
        next_object/2
    ]).


next_object(Object, 0) :-
    objects_not_handeled(PossibleObjects),
    tf_lookup_transform('base_footprint', 'map', Pose(CurrentPosition, _)),
    all_objects_cost_benefit_ratios(CurrentPosition, PossibleObjects, CBRatios),
    min_member([Object, _], CBRatios).


next_object(Object, 1) :-
    current_object(CurrentObject),
    ask(triple(CurrentObject, hsr_objects:'hasSuccessorObject', Object)).


next_object(Object, 1) :-
    objects_not_handeled(PossibleObjects),
    length(PossibleObjects, Count),
    Count < 6,
    findall(Permutation, permutation(PossibleObjects, Permutation), PossiblePermutations),
    findall([Permutation, CBRatio],
    (
        permutation_cost_benefit_ratio(Permutation, CBRatio)
    ),
    CBRatios),
    min_member([Permutation, _], CBRatios),
    build_next_object_graph(Permutation),
    nth0(0, Permutation, Object).


build_next_object_graph(Permutation) :-
    forall(nextto(Object, SuccessorObject, Permutation), 
        tell(triple(Object, hsr_objects:'hasSuccessorObject', SuccessorObject))).


remove_next_object_graph :-
    forall(triple(Object, hsr_objects:'hasSuccessorObject', SuccessorObject), 
        tripledb_forget(Object, hsr_objects:'hasSuccessorObject', SuccessorObject)).


current_object(Object) :-
    is_suturo_object(Object),
    handeled(Object),
    ask(triple(Object, hsr_objects:'hasSuccessorObject', SuccessorObject)),
    not handeled(SuccessorObject).
    

permutation_cost_benefit_ratio(Permutation, TotalCBRatio) :-
    findall(CBRatio,
    (
        nextto(Object, SuccessorObject, Permutation),
        object_pose(Object, [_, _,ObjectPosition, _]),
        findall(PossibleObject, (member(PossibleObject, Permutation), not same_as(PossibleObject, Object)), PossibleObjects),
        all_objects_normalized_costs_and_benefits(ObjectPosition, PossibleObjects, CBRatios),
        member([SuccessorObject, CBRatio], CBRatios)
    ),
    PermutationCBRatios),
    sumlist(PermutationCBRatios, TotalCBRatio).    


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


object_benefit(Object, Benefit) :-
    ask(triple(Object, hsr_objects:'ClassConfidenceValue', Benefit)).


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
    object_pose(Object, [_, _,ObjectPosition, _]),
    object_at_predefined_location(Object, RoomType, FurnitureType),
    surface_at_predefined_location(GoalSurface, RoomType, FurnitureType),
    surface_pose_in_map(Object, [SurfacePosition, _]),
    euclidean_distance(ObjectPosition, SurfacePosition, Distance).

