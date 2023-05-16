:- module(next_object,
    [
        next_object/1,
        distance_to_object/2
        
    ]).
:- use_module(library('model/locations/spatial_computations')).
:- use_module(library('model/locations/actual_locations')).
%% next_object(+Object) is semidet.
%
% Choose the next best object to grasp.
next_object(Object):-
%triple(Object, suturo:hasDataSource, perception),
    fail.
% try if object is already handled
% try if not handled objects are misplaced
% then we have possible objects to choose the next best object
% get robots position
% calculate cost for robot position and possible objects
% calculate benefit for possible objects
% return object bonus
% calculate cost benefit ratio
% choose the object with the highest ratio


%% object_bendefits(+Object, -ObjectBenefits) is semidet.
%
%find benefits for all objects 
objects_bendefits(Objects, ObjectBenefits) :-
    fail.

%% object_benefit(+Object, -Benefit) is semidet.
%
% Calculate object benefit (class confidence of an object)
object_benefit(Object, Benefit):-
    fail.
% calculate confidence class value(measure a confidence that a robot has about the recognition of objects)


%% object_costs(+OriginalPosition, +Objects, -ObjectCosts) is semidet.
%
%find costs for all objects
object_costs(OriginalPosition, Objects, ObjectCosts) :-
    fail.


%% object_cost(+OriginPosition, +Object, -Costs) is semidet.
%
% Calculate object cost.
object_cost(OriginPosition, Object, Costs):-
    fail.
% Calculate distance the point we get for an object and the distance we have to go.

%% object_bonus(+OriginalPosition, +Object, -Bonus) is semidet.
%
% return object bonus.
object_bonus(OriginalPosition, Object, Bonus):-
    fail.
%initialize object bonus

%% distance_to_go(+OriginPosition, +Object, -Distance) is semidet.
%
% Calculate distance we have to go.
distance_to_go(OriginPosition, Object, Distance):-
    fail.
% calculate distance to object and distance to goal location


%% distance_to_object(+OriginPosition, +Object, -Distance)is semidet.
%
% Calculate distance the point we get for an object.
distance_to_object(Object, Distance):-
    kb_call(is_at(Object,[map,ObjectLocation,_])),
    robot_location(RobotLocation),
    euclidean_distance(ObjectLocation, RobotLocation, Distance).
% calcluta euclidian distance between robot and an object



%%distance_to_goal_location(+Object, -Distance) is semidet.
%
% Calculate distance between object position and goal position
distance_to_goal_location(Object, Distance):-
    fail.


%%object_goal_location(+Object, -GoalPosition) is semidet.
%
% determines object goal location
object_goal_location(Object, GoalPosition) :-
    fail.











