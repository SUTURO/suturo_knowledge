:- module(grasping,[
    attach_object_to_gripper/1,
    object_pose_to_grasp_from/2
]).


:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).

:- use_module(library('ros/marker/marker_plugin'), [marker_message_new/3, republish/0]).

:- use_module(library('locations/actual_locations'), [forget_object_at_location/1]).
:- use_module(library('locations/spatial_comp'), 
    [
        hsr_lookup_transform/4,
        surface_dimensions/4,
        surface_front_edge_center_pose/2
    ]).
:- use_module(library('gripper/gripper_info'), [gripper/1]).

%% surface_pose_to_perceive_from(Surface, [[XPos,YPos,0],Rotation]) is nondet.
%
% Returns the position to take on when perceiving from surface
%
% @param BestObj the variable to be filled with the next object
%
surface_pose_to_perceive_from(Surface, [[XPos,YPos,0],Rotation]):-
    has_urdf_name(Surface, SurfaceLink),
    surface_dimensions(Surface,X,Y,_),
    %% We want the robot to see the whole table, but be as near to it as possible.
    %% The viewing angle of the hsrb is a bit more than 35° to either side, so 70° in total.
    %% To calculate the distance to the table, we now can use formulas for a triangle with a right angle

    %% hsb
    %% |\<-- alpha
    %% | \
    %% B  \
    %% |   \
    %% |    \
    %% --A----
    %% table

    %% tan(35°) = 0,7
    %% tan(alpha) = A/B
    %% B = A/tan(alpha)
    %% Y is twice the length of the half
    B is (Y / 1.4),
    %% B is the minimum distance from the from to see the whole from side with a viewing angle of 35°
    HalfX is X / 2,
    XOffset is -(HalfX + B),
    %% The robot should be at least 0.6m away from the edge of the surface in any case.
    (XOffset >= -0.6  - HalfX ->
      XOffsetUsed is -0.6  - HalfX;
      XOffsetUsed is XOffset),
    tf_transform_point(SurfaceLink, map, [XOffsetUsed, 0, 0], [XPos,YPos,_]),
    tf_lookup_transform('map', SurfaceLink, pose(_,Rotation)).

%% attach_object_to_gripper(Objects) is nondet.
%
% Attaches the object to the gripper
%
% @param Objects the object which will be grasped
%
attach_object_to_gripper(Object) :-
    ros_info("Call Attach object to gripper"),
    ros_info(Object),
    forget_object_at_location(Object),
    gripper(Gripper),
    has_location(Object, ObjectLocation),
    tell(triple(ObjectLocation, soma:'isSupportedBy', Gripper)),
    object_tf_frame(Object,ObjectFrame),
    hsr_lookup_transform(Gripper, ObjectFrame, PoseTrans, PoseRota),
    tell(is_at(Object, [Gripper, PoseTrans, PoseRota])),
    republish, republish.

%% object_pose_to_grasp_from(Object,[[XPose,YPose,0], Rotation]) is nondet.
%
% Returns the position to take on grasping an object
%
% @param Object the to be grasped
% @param [[XPose,YPose,0], Rotation] the position of the object
%
object_pose_to_grasp_from(Object,[[XPose,YPose,0], Rotation]):-
    object_supported_by_surface(Object,Surface),
    has_urdf_name(Surface,Name),
    surface_front_edge_center_pose(Surface,[_, Rotation]),
    object_tf_frame(Object,F),
    tf_lookup_transform(Name, F, pose([_,Y,_],_)),
    surface_dimensions(Surface, Depth, _, _),
    Offset is -(Depth / 2 + 0.5),
    tf_transform_point(Name, map, [Depth, Y,0], [XPose,YPose,_]).







