% furniture informations
:- module(furniture_info,
	[
		furniture_rel_pose(r,+,-),
        has_robocup_name(r,?)
	]).

:- use_module(library('util/math'),
	[
		deg_to_rad/2
	]).

%% furniture_rel_pose(+Furniture, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to a furniture based on the type of relation.
%
% @param Furniture The furniture to which the position is relative to.
% @param Type The type of relation. It can be "perceive" or "interact".
% @param PoseStamped The position relative to the furniture.
%
furniture_rel_pose(Furniture, Type, PoseStamped) :-
 	% if the type is "perceive", call the furniture_rel_pose_perceive/2 predicate
 	(Type = perceive
	 ->	furniture_rel_pose_perceive(Furniture, PoseStamped)
    % if the type is "interact", call the furniture_rel_pose_interact/2 predicate
	; Type = interact
	 ->	furniture_rel_pose_interact(Furniture, PoseStamped)
	% if the type is not defined, return an error message
	; ros_error('The furniture_rel_pose type ~w is not defined.', [Type]),
	  false
	).

furniture_rel_pose_perceive(Furniture, PoseStamped) :-
	% Get the PoseStamped of the Furniture
	object_pose(Furniture, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.7,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% deg_to_rad(70, CameraViewAngle)

	% TODO: Calculate perceiving position relative to the Furniture
	%
	% HSR
    % |\<-- alpha
    % | \
    % B  \
    % |   \
    % |    \
    % ---A---
    % Furniture
	% % B = A / tan(alpha)

furniture_rel_pose_interact(Furniture, PoseStamped) :-
	% Get the PoseStamped of the Furniture
	object_pose(Furniture, [Frame, [X,Y,Z], Rotation]),
	XNew is X - 0.5,
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].
	% TODO: Calculate interacting position relative to the Furniture

%% has_robocup_name(?Furniture, ?Name) is nondet.
%
% get the knowledge_role assigned to a furniture in the semantic map.
% make sure that the knowledge_role in there matches the robocup name.
has_robocup_name(Furniture,Name) ?+>
    holds(Furniture,suturo:hasRobocupName,Name).
