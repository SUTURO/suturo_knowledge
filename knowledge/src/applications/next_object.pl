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


:- use_module(library('model/objects/object_manipulation'), 
    [
        objects_not_handeled/1,
        handeled/1
    ]).
:- use_module(library('model/objects/object_info'), 
    [
        is_suturo_object/1,
        hsr_existing_objects/1
    ]).
:- use_module(library('locations/misplaced'), [is_misplaced/1]).
:- use_module(library('locations/spatial_comp'), 
    [
        object_pose/2,
        surface_pose_in_map/2,
        compareDistances/3,
        euclidean_distance/3
    ]).
:- use_module(library('locations/predefined_locations'), 
    [
        object_at_predefined_location/3,
        surface_at_predefined_location/3
    ]).


next_object(Object, 0) :-
    objects_not_handeled(NotHandledObjects),
    ros_info("Objects not handled"),
    ros_info(NotHandledObjects),
    findall(NotHandledObject, 
    (
        member(NotHandledObject, NotHandledObjects),
        ros_info("Object"),
        ros_info(NotHandledObject),
        is_misplaced(NotHandledObject),
        ros_info("Is Misplaced")
    ), 
    PossibleObjects),
    ros_info("Possible Objects for next object"),
    ros_info(PossibleObjects),
    once(is_at(base_footprint, ['map', RobotPosition, _]) -> true; ros_info("next_object: is_at failed.")),
    objects_costs(RobotPosition, PossibleObjects, Costs),
    ros_info("Object costs"),
    ros_info(Costs),
    objects_benefits(PossibleObjects, Benefits),
    ros_info("Object Benefits"),
    ros_info(Benefits),
    %max_member([_, NormalizationConstant], Costs),
    findall([CBRatio, Object],
    (
	member([Object, BenefitAtom], Benefits),
	atom_number(BenefitAtom, Benefit),
	ros_info("Benefit is"),
	ros_info(Benefit),
	member([Object, Cost], Costs),
	ros_info("Cost is"),
	ros_info(Cost),
	%CBRatio is Benefit / (Cost / NormalizationConstant)
	CBRatio is Benefit / Cost,
	ros_info("CBRatio is"),
	ros_info(CBRatio)
    ),
    ObjectCBRatios),
    list_to_set(ObjectCBRatios, ObjectCBRatiosSet),
    max_member([_, Object], ObjectCBRatios),
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
        forget_tour,
        next_object(CheapestObject, 0),
        tell(has_type(StartPosition, hsr_rooms:'StartMark')),
        is_at(base_footprint, ['map', RobotPosition, RobotRotation]),
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


next_object(Object, 2) :-
    objects_not_handeled(ObjectsNotHandled),
    findall(Obj, (member(Obj, ObjectsNotHandled), is_misplaced(Obj)), Objects),
    predsort(compareDistances, Objects, SortedObjs),
    nth0(0, SortedObjs, Object).


forget_tour :-
    ros_info("Call forget tour"),
    forall(triple(Object, dul:'follows', _), tripledb_forget(Object, dul:'follows', _)),
    forall(has_type(StartMark, hsr_rooms:'StartMark'), tripledb_forget(StartMark, _, _)).


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
    %max_member([_, _, NormalizationConstant], InsertionCosts),
    findall([CBRatio, Object, SubTour],
    (
        member([Object, SubTour, Costs], InsertionCosts),
        object_benefit(Object, BenefitAtom),
        atom_number(BenefitAtom, Benefit),
        %CBRatio is Benefit / (Costs / NormalizationConstant)
        %CBRatio is Benefit / Costs
        CBRatio is Costs
    ), 
    CBRatios),
    list_to_set(CBRatios, CBRatiosSet),
    %max_member([_, Object, [Object1, Object2]], CBRatiosSet),
    min_member([_, Object, [Object1, Object2]], CBRatiosSet),
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
        (
	    object_costs(OriginPosition, Object, Costs)-> true ;
	    format(string(MSG), "object_costs(~w, ~w, ObjectCosts) failed", [OriginPosition, Objects]),
	    ros_warn(MSG),
	    fail()
	)
    ),
    ObjectCosts).


object_costs(OriginPosition, Object, Costs) :-
    distance_to_go(OriginPosition, Object, DistanceToGo),
    %robot_velocity(RobotVelocity),
    %Costs is DistanceToGo / RobotVelocity.
    Costs is DistanceToGo.


distance_to_go(OriginPosition, Object, Distance) :-
    (distance_to_object(OriginPosition, Object, DistanceToObject) -> true;
     format(string(MSG), "distance_to_object(~w, ~w, DistanceToObject) failed", [OriginPosition, Object]),
     ros_warn(MSG),
     fail()
    ),
    (distance_to_goal_location(Object, DistanceToLocation) -> true;
     format(string(MSG), "distance_to_goal_location(~w, DistanceToLocation) failed", [OriginPosition]),
     ros_warn(MSG),
     fail()
    ),
    Distance is DistanceToObject + DistanceToLocation.


distance_to_object(OriginPosition, Object, Distance) :-
    (object_pose(Object, [_, _,[X, Y, _], _]) -> true;
     format(string(MSG), "object_pose(~w, [_, _,[X, Y, _], _]) failed", [Object]),
     ros_warn(MSG),
     fail()
    ),
    euclidean_distance(OriginPosition, [X, Y, 0], Distance).


distance_to_goal_location(Object, Distance) :-
    (object_goal_location(Object, GoalPosition) -> true;
     format(string(MSG), "object_goal_location(~w, GoalPosition) failed", [Object]),
     ros_warn(MSG),
     fail()
    ),
    object_pose(Object, [_, _,ObjectPosition, _]),
    euclidean_distance(ObjectPosition, GoalPosition, Distance).


object_goal_location(Object, GoalPosition) :-
    object_pose(Object, [_, _,ObjectPosition, _]),
    (object_at_predefined_location(Object, RoomType, FurnitureType)-> true;
     format(string(MSG), "object_at_predefined_location(~w, RoomType, FurnitureType) failed", [Object]),
     ros_warn(MSG),
     fail()
    ),
    (once(surface_at_predefined_location(GoalSurface, RoomType, FurnitureType))-> true;
     format(string(MSG), "once(surface_at_predefined_location(GoalSurface, ~w, ~w) failed", [RoomType, FurnitureType]),
     ros_warn(MSG),
     fail()
    ),
    (has_urdf_name(GoalSurface, Name) -> true;
     format(string(MSG), "has_urdf_name(~w, Name) failed", [GoalSurface]),
     ros_warn(MSG),
     fail()
    ),
    (surface_pose_in_map(GoalSurface, [GoalPosition, _]) -> true;
     format(string(MSG), "surface_pose_in_map(~w, [GoalPosition, _]) failed", [GoalSurface]),
     ros_warn(MSG),
     fail()
    ).


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











    

