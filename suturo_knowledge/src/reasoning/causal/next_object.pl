:- module(next_object,
    [
        next_object(-),
        objects_benefits(+, -),
        object_benefit(r, -),
        object_costs(+, -),
        object_cost(r, -),
        objects_bonus(+, -),
        object_bonus(r, -),
        distance_to_go(r, -),
        distance_to_object(r, -),
        distance_to_destination_location(r, -)
    ]).

:- use_module(library('reasoning/metric/size')).
:- use_module(library('reasoning/spatial/distance')).

%% next_object(-Object) is nondet.
%
% Chooses the next best object to pick based on the current challenge
%
% @param Object The next best object to pick
%
next_object(Object) :-
    has_type(_, suturo:'StoringGroceries'),
    !,
    next_object_storing_groceries(Object).
next_object(Object) :-
    has_type(_, suturo:'ServeBreakfast'),
    !,
    next_object_storing_groceries(Object).
    % TODO: Implement algorithm for ServeBreakfast
    % next_object_serve_breakfast(Object).
next_object(Object) :-
    has_type(_, suturo:'CleanTheTable'),
    !,
    next_object_clean_the_table(Object).
next_object(_Object) :-
    ros_error('next_object: no challenge initialized').

next_object_storing_groceries(NextObject) :-
    objects_not_handled(NothandledObjects),
    ros_info('Not handled objects: ~w', [NothandledObjects]),
    find_next_object_storing_groceries(NothandledObjects, NextObject),
    set_object_handled(NextObject),
    !.

%% find_next_object_storing_groceries(+Objects, -NextObject) is nondet.
%
% Chooses the next best object to pick for the StoringGroceries challenge.
%
% @param Objects The objects to choose from
% @param NextObject The next best object to pick
%
find_next_object_storing_groceries(NothandledObjects, NextObject) :-
    (
        % First try to find objects with a bonus of 0
        setof([CbRatio, Object],
            (
                member(Object, NothandledObjects),
                object_bonus(Object, 0),
                object_benefit(Object, Benefit),
                object_cost(Object, Cost),
                CbRatio is Benefit / Cost
            ),
            SortedPairs0)
    ->
        SortedPairs = SortedPairs0
    ;
        % If no objects with a bonus of 0 exist, find objects with any bonus
        setof([CbRatio, Object],
            (
                member(Object, NothandledObjects),
                object_benefit(Object, Benefit),
                object_cost(Object, Cost),
                CbRatio is Benefit / Cost
            ),
            SortedPairs)
    ),
    % The last element of SortedPairs is the pair with the highest CbRatio
    last(SortedPairs, [_BestCbRatio, NextObject]).

%% next_object_clean_the_table(-NextObject) is nondet.
%
% Chooses the next best object to pick for the CleanTheTable challenge.
% TODO: Implement optimized algorithm for CleanTheTable
% This includes then handling of small cutlery like knife, spoons and forks and the DishwasherTab.
%
% @param NextObject The next best object to pick
%
next_object_clean_the_table(NextObject) :-
    ignore(( has_type(Object, soma:'DishwasherTab'),
	     set_object_handled(Object))),
    objects_not_handled(NothandledObjects),
    findall([Object, CbRatio],
        (
            member(Object, NothandledObjects),
            object_bonus(Object, 500),
            object_benefit(Object, Benefit),
            object_cost(Object, Cost),
            CbRatio is Benefit / Cost
        ),
        ObjectsAndCbRatio0
        ),
        (
            ObjectsAndCbRatio0 ==[]
            ->
            findall([Object,CbRatio],
                (member(Object,NothandledObjects),
                object_benefit(Object, Benefit),
                object_cost(Object,Cost),
                CbRatio is Benefit/Cost),
                ObjectsAndCbRatio)
            ;   ObjectsAndCbRatio0 = ObjectsAndCbRatio
        ),
        (ObjectsAndCbRatio == []
            ->
            has_type(NextObject, soma:'DishwasherTab')
            ;
            find_best_object(ObjectsAndCbRatio, NextObject),
            set_object_handled(NextObject)
        ),
        !.

%% find_best_object(+Objects, -BestObject) is semidet.
%
% Finds the the object with the highest benefit to cost ratio.
%
% @param ObjectsAndCBRatio List of object and cost benifit ratio pairs
% @param BestObject The best object to pick
%
find_best_object(ObjectsAndCbRatio, BestObject) :-
    maplist(nth0(1), ObjectsAndCbRatio, CbRatios),
    max_list(CbRatios, MaxCbRatio),
    member([BestObject, MaxCbRatio], ObjectsAndCbRatio).

%% object_bendefits(+Objects, -ObjectsBenefits) is semidet.
%
% Finds the benefit for all obects.
%
% @param Objects The objects to calculate the benefit for
% @param ObjectsBenefits The benefits for the given objects
%
objects_benefits(Objects, ObjectsBenefits) :-
    findall([Object,Benefit],
        (
            member(Object,Objects),
            object_benefit(Object,Benefit)
        ),
    ObjectsBenefits).

%% object_benefit(+Object, -Benefit) is semidet.
%
% Calculates the object benefit.
% The higher the detection confidence value by Perception the higher the benefit.
%
% @param Object The object to calculate the benefit for
% @param Benefit The benefit for the given object
%
object_benefit(Object, Benefit):-
    ask(triple(Object, suturo:'hasConfidenceValue', Benefit)).

%% object_costs(+Objects, -ObjectsCosts) is semidet.
%
% Finds the cost for all objects
%
% @param Objects The objects to calculate the cost for
% @param ObjectCosts The costs for the given objects
%
object_costs(Objects, ObjectsCosts) :-
    findall([Object,Cost],
        (
            member(Object,Objects),
            object_cost(Object,Cost)
        ),
        ObjectsCosts).


%% object_cost(+Object, -Costs) is semidet.
%
% Calculates the cost to handle the given object.
%
% @param Object The object to calculate the cost for
% @param Cost The cost for the given object
%
object_cost(Object, Cost) :-
    distance_to_go(Object, DistanceToGo),
    Cost is DistanceToGo.

%% objects_bonus(+Objects, -ObjectsBonus) is semidet.
%
% Finds the bonus for all objects
%
% @param Objects The objects to calculate the bonus for
% @param ObjectsBonus The boni for the given objects
%
objects_bonus(Objects, ObjectsBonus):-
    findall([Object, Bonus],
        (
            member(Object, Objects),
            object_bonus(Object, Bonus)
        ),
        ObjectsBonus).

%% object_bonus(+Object, -Bonus) is det.
%
% Returns the object bonus for the given object according to the RoboCup@Home rulebook.
%
% @param Object The object to calculate the bonus for
% @param Bonus The bonus for the given object
%
object_bonus(Object, Bonus):-
    is_tiny(Object)
    -> Bonus = 500
    ;  Bonus = 0.

%% distance_to_go(+Object, -Distance) is semidet.
%
% Calculates the distance the robot would need to move to reach the object and then bring it to its destination location
%
% @param Object The object to calculate the distance for
% @param Distance The overall distance from the robot to the objects and its destination location
%
distance_to_go(Object, Distance) :-
    distance_to_object(Object, DistanceToObject),
    distance_to_destination_location(Object, DistanceToDestinationLocation),
    Distance is DistanceToObject + DistanceToDestinationLocation.

%% distance_to_object(+Object, -Distance) is semidet.
%
% Calculates the distance the distance between the robot and the object location
%
% @param Object The object to calculate the distance to
% @param Distance The distance between the robot and the object location
distance_to_object(Object, Distance) :-
    get_urdf_origin(Origin),
    kb_call(is_at(Object,[Origin, ObjectLocation, _])),
    robot_location(RobotLocation),
    euclidean_distance(ObjectLocation, RobotLocation, Distance).


%% distance_to_destination_location(+Object, -Distance) is semidet.
%
% Calculates the distance between object location and destination location
%
% @param Object The object to calculate the distance for
% @param Distance The distance between the object and the destination location
%
distance_to_destination_location(Object, Distance) :-
    get_urdf_origin(Origin),
    kb_call(is_at(Object, [Origin, ObjectLocation, _])),
    has_type(Object, Class),
    % TODO use actual destination location
    predefined_destination_location(Class, DestinationLocationObject),
    kb_call(is_at(DestinationLocationObject, [Origin, DestinationLocation, _])),
    euclidean_distance(ObjectLocation, DestinationLocation, Distance).
