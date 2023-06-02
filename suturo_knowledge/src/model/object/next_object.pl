:- module(next_object,
    [
        next_object/1,
        objects_benefits/2,
        object_benefit/2,
        object_costs/2,
        object_cost/2,
        objects_bonus/2,
        object_bonus/2,
        distance_to_go/2,
        distance_to_object/2,
        distance_to_goal_location/2
        
    ]).
:- use_module(library('model/locations/spatial_computations')).
:- use_module(library('model/locations/actual_locations')).
%% next_object(+Object) is semidet.
%
% Choose the next best object to grasp.
next_object(Object):-
% try if object is already handled
% try if not handled objects are misplaced
% then we have possible objects to choose the next best object
% calculate cost for robot position and possible objects
% calculate benefit for possible objects
% return object bonus
% calculate cost benefit ratio
% choose the object with the highest ratio
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


%% object_bendefits(+Object, -ObjectBenefits) is semidet.
%
%find benefits for all objects 
objects_benefits(Objects, ObjectBenefits) :-
    findall([Object,Benefit],
        (
            member(Object,Objects),
            object_benefit(Object,Benefit)
        ),
    ObjectBenefits).

%% object_benefit(+Object, -Benefit) is semidet.
%
% Calculate object benefit (class confidence of an object)
object_benefit(Object, Benefit):-
    ask(triple(Object, suturo:'hasConfidenceValue', Benefit)).
% calculate confidence class value(measure a confidence that a robot has about the recognition of objects)


%% object_costs(+Object, -ObjectCosts) is semidet.
%
%find costs for all objects
object_costs(Objects, ObjectCosts) :-
    findall([Object,Cost],
        (
            member(Object,Objects),
            object_cost(Object,Cost)
        ),
    ObjectCosts).


%% object_cost(+Object, -Costs) is semidet.
%
% Calculate object cost.
object_cost(Object, Cost):-
    distance_to_go(Object,DistanceToGo),
    Cost is DistanceToGo.
% Calculate distance the point we get for an object and the distance we have to go.

objects_bonus(Objects, ObjectBonus):-
    findall([Object,Bonus],
        (
            member(Object,Objects),
            object_bonus(Object,Bonus)
        ),
    ObjectBonus).

%% object_bonus(+Object, -Bonus) is semidet.
%
% return object bonus.
object_bonus(Object, Bonus):-
    tiny_object(Object)
    -> Bonus=500
    ;  Bonus=0.
%initialize object bonus

%% distance_to_go(+Object, -Distance) is semidet.
%
% Calculate distance we have to go.
distance_to_go(Object, Distance):-
    distance_to_object(Object,DistanceToObject),
    distance_to_goal_location(Object, DistanceToGoalLocation),
    Distance is DistanceToObject + DistanceToGoalLocation.
% calculate distance to object and distance to goal location


%% distance_to_object(+Object, -Distance)is semidet.
%
% Calculate distance the point we get for an object
distance_to_object(Object, Distance):-
    kb_call(is_at(Object,[map,ObjectLocation,_])),
    robot_location(RobotLocation),
    euclidean_distance(ObjectLocation, RobotLocation, Distance).
% calcluta euclidian distance between robot and an object



%%distance_to_goal_location(+Object, -Distance) is semidet.
%
% Calculate distance between object position and goal position
distance_to_goal_location(Object, Distance):-
    kb_call(is_at(Object,[map,ObjectLocation,_])),
    GoalLocation = [1, 2, 0], % Test value until predefined location implemented
    euclidean_distance(ObjectLocation,GoalLocation,Distance).


%%object_goal_location(+Object, -GoalPosition) is semidet.
%
% determines object goal location
object_goal_location(Object, GoalPosition) :-
    fail.











