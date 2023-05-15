% container informations
:- module(container_info,
	[
		container_rel_pose(r,+,-)
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
    ; Type = destination
     ->	container_rel_pose_destination(Container, PoseStamped)
    ; Type = grasp
     ->	container_rel_pose_grasp(Container, PoseStamped)
    % if the type is not defined, return an error message
    ; ros_error('The container_rel_pose type ~w is not defined.', [Type]),
      false
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

%% container_rel_pose_destination(+Container, -PoseStamped) is semidet.
%
% Gets the destination pose of the cereal box.
%
% @param PoseStamped The destination pose of the cereal box.
%
container_rel_pose_destination(Container, PoseStamped) :-
    has_type(Container, soma:'CerealBox'),
    has_urdf_name(Destination, 'shelf:shelf:shelf_base_center'),
    object_pose(Destination, [Frame, [X,Y,Z], Rotation]),
    YNew is Y - 0.1,
    ZNew is Z + 0.51,
    PoseStamped = [Frame, [X,YNew,ZNew], Rotation].

container_rel_pose_grasp(Container, PoseStamped) :-
    has_type(Container, soma:'CerealBox'),
    has_urdf_name(Destination, 'tall_table:table:table_front_edge_center'),
    object_pose(Destination, [Frame, [X,Y,Z], _]),
    XNew is X + 0.05,
    YNew is Y + 0.1,
    ZNew is Z + 0.08,
    PoseStamped = [Frame, [XNew,YNew,ZNew], [-0.0029949, 0.0014875, -0.0784591, 0.9969117]].