:- module(next_object,
    [
        next_object(-),
        next_object1(-),
        next_object2(-),
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

%% next_object(-Object) is semidet.
%
% Choose the next best object to grasp.
%
% try if object is already handled
% try if not handled objects are misplaced
% then we have possible objects to choose the next best object
% calculate cost for robot position and possible objects
% calculate benefit for possible objects
% return object bonus
% calculate cost benefit ratio
% choose the object with the highest ratio
%
next_object(Object):-
    objects_not_handled(NothandledObjects),
    findall([Object,CBRatio],
        (member(Object,NothandledObjects),
        object_bonus(Object, 0),
        object_benefit(Object, Benefit),
        object_cost(Object,Cost),
        CBRatio is Benefit/Cost),
        ObjectCBRatio0),
        (ObjectCBRatio0 == []
        -> 
        findall([Object,CBRatio],
            (member(Object,NothandledObjects),   
            object_benefit(Object, Benefit),
            object_cost(Object,Cost),
            CBRatio is Benefit/Cost),
            ObjectCBRatio)
        ;   ObjectCBRatio0 = ObjectCBRatio
        ),
    maplist(nth0(1), ObjectCBRatio, CBRatios),
    max_list(CBRatios, MaxCBRatio),
    member([Object, MaxCBRatio], ObjectCBRatio),
    set_object_handled(Object),
    !.

next_object1(Object):-
    set_object_handled('http://www.ease-crc.org/ont/SOMA.owl#DishwasherTab_GUSPBNMR'),
    objects_not_handled(NothandledObjects),
    findall([Object,CBRatio],
        (member(Object,NothandledObjects),
        object_bonus(Object, 500),
        object_benefit(Object, Benefit),
        object_cost(Object,Cost),
        CBRatio is Benefit/Cost),
        ObjectCBRatio0),
        (ObjectCBRatio0 == []
        -> 
        findall([Object,CBRatio],
            (member(Object,NothandledObjects),   
            object_benefit(Object, Benefit),
            object_cost(Object,Cost),
            CBRatio is Benefit/Cost),
            ObjectCBRatio)
        ;   ObjectCBRatio0 = ObjectCBRatio
        ),
        (ObjectCBRatio == []
            ->
            Object = 'http://www.ease-crc.org/ont/SOMA.owl#DishwasherTab_GUSPBNMR'
            ;
            maplist(nth0(1), ObjectCBRatio, CBRatios),
            max_list(CBRatios, MaxCBRatio),
            member([Object, MaxCBRatio], ObjectCBRatio),
            set_object_handled(Object)
        ),
        !.



next_object2(Object):-
    set_object_handled('http://www.ease-crc.org/ont/SOMA.owl#DishwasherTab_GUSPBNMR'),
    objects_not_handled(NothandledObjects),
    findall([Object,CBRatio],
        (member(Object,NothandledObjects),
        object_bonus(Object, 0),
        object_benefit(Object, Benefit),
        object_cost(Object,Cost),
        CBRatio is Benefit/Cost),
        ObjectCBRatio0),
        (ObjectCBRatio0 == []
        -> 
        findall([Object,CBRatio],
            (member(Object,NothandledObjects),   
            object_benefit(Object, Benefit),
            object_cost(Object,Cost),
            CBRatio is Benefit/Cost),
            ObjectCBRatio)
        ;   ObjectCBRatio0 = ObjectCBRatio
        ),
        (ObjectCBRatio == []
            ->
            Object = 'http://www.ease-crc.org/ont/SOMA.owl#DishwasherTab_GUSPBNMR'
            ;
            maplist(nth0(1), ObjectCBRatio, CBRatios),
            max_list(CBRatios, MaxCBRatio),
            member([Object, MaxCBRatio], ObjectCBRatio),
            set_object_handled(Object)
        ),
        !.

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
    distance_to_go(Object,DistanceToGo),
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
    kb_call(is_at(Object,[Origin, ObjectLocation, _])),
    DestinationLocation = [1, 2, 0], % TODO get goal location from knowledge base
    euclidean_distance(ObjectLocation, DestinationLocation, Distance).