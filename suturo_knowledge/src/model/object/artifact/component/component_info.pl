% component informations
:- module(component_info,
	[
		component_rel_pose(r,+,-)
	]).

%% component_rel_pose(+Component, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to a component based on the type of relation.
%
% @param Component The component to which the position is relative to.
% @param Type The type of relation. It can be "perceive" or "interact".
% @param PoseStamped The position relative to the component.
%
component_rel_pose(Component, Type, PoseStamped) :-
    % if the type is "perceive", call the component_rel_pose_perceive/2 predicate
    (Type = perceive
     ->	component_rel_pose_perceive(Component, PoseStamped)
    % if the type is "interact", call the component_rel_pose_interact/2 predicate
    ; Type = interact
     ->	component_rel_pose_interact(Component, PoseStamped)
    ; Type = grasp
     ->	component_rel_pose_grasp(Component, PoseStamped)
    % if the type is not defined, return an error message
    ; ros_error('The component_rel_pose type ~w is not defined.', [Type]),
      false
    ).

component_rel_pose_perceive(Component, PoseStamped) :-
	% Get the PoseStamped of the component
	object_pose(Component, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.4,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% deg_to_rad(70, CameraViewAngle)
	
	% TODO: Calculate perceiving position relative to the component
	% 
	% HSR
    % |\<-- alpha
    % | \
    % B  \
    % |   \
    % |    \
    % ---A---
    % component
	% % B = A / tan(alpha)

component_rel_pose_interact(Component, PoseStamped) :-
	% Get the PoseStamped of the component
	object_pose(Component, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.5,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% TODO: Calculate interacting position relative to the component

component_rel_pose_grasp(Component, PoseStamped) :-
    has_type(Component, soma:'Drawer'),
    has_urdf_name(Destination, 'drawer:drawer:drawer_front_top'),
    object_pose(Destination, [Frame, [X,Y,Z], Rotation]),
    XNew is X - 0.3,
    ZNew is Z - 0.078,
    PoseStamped = [Frame, [XNew,Y,ZNew], Rotation].