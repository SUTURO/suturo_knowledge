% container informations
:- module(container_info,
	[
		container_rel_pose/3
	]).

%% container_rel_pose(+Container, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to a container based on the type of relation.
%
% @param Container The container to which the position is relative to.
% @param Type The type of relation. It can be "perceive" or "interact".
% @param PoseStamped The position relative to the container.
%
container_rel_pose(Container, Type, PoseStamped) :-
    % if the type is "perceive", call the container_rel_pose_perceive/2 predicate
    (Type = perceive
    ->	container_rel_pose_perceive(Container, PoseStamped)
   % if the type is "interact", call the container_rel_pose_interact/2 predicate
   ; Type = interact
    ->	container_rel_pose_interact(Container, PoseStamped)
   % if the type is not defined, return an error message
   ; ros_error('The container_rel_pose type ~w is not defined.', [Type])
    ).

container_rel_pose_perceive(Container, PoseStamped) :-
	% Get the PoseStamped of the container
	object_pose(Container, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.4,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% deg_to_rad(70, CameraViewAngle)
	
	% TODO: Calculate perceiving position relative to the container
	% 
	% HSR
    % |\<-- alpha
    % | \
    % B  \
    % |   \
    % |    \
    % ---A---
    % container
	% % B = A / tan(alpha)

container_rel_pose_interact(Container, PoseStamped) :-
	% Get the PoseStamped of the container
	object_pose(Container, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.5,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% TODO: Calculate interacting position relative to the container