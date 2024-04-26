% furniture informations
:- module(furniture_info,
	[
		furniture_rel_pose(r,+,-),
        has_robocup_name(r,?),
		div_parts(+, -),
		div_partss(+,-),
		divide_next(+,+,-),
		opt_nav_poses(+, -)
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
	opt_nav_poses(Furiniture, Middle),
    PoseStamped = [Frame, [XNew,Y,Z], Rotation].

% optional navigation poses when interacting with furniture items
% Middle is the new x position
%% opt_nav_poses
opt_nav_poses(Furniture, Size):-
	object_shape_workaround(Furniture, _, ShapeTerm,_,_),
	dir_size('-y', ShapeTerm, Size).
	%try_divide(Size, Middle).
		
try_divide(Size, Middle) :-
	(   div_parts(Size, Middle)
	;   div_partss(Size, Middle)  
	).
		
	div_parts(Size, Middle) :-
		Size > 0, 
		Size =< 0.8,
		Middle is Size / 2.0.
		
	div_partss(Size, Midpoints):-
		Size > 0.8,
		writeln("1."),
		Middle is 0.4,
		writeln("2."),
		divide_next(Size, 0.8, [Middle]). % add 40 in list

	divide_next(Size, Save, Midpoints):-
		(	Size > Save,
			writeln("3.")
		->	NSave is Save + 0.7,
			Middle is Save - (0.7/ 2.0),
			divide_next(Size, NSave, Midpoints)
		;	Midpoints = 0.7,
			writeln("4. ~w", [Midpoints])
		).


	

% divides the longest side of the furniture into parts 
% parts have size of the plane that the robots camera can perceive w/ head rotation
%% divide(+Size, -Middle)

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
