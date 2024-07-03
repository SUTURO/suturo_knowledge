% furniture information
:- module(furniture_info,
	[
		furniture_rel_pose(r,+,-),
        has_robocup_name(r,?),
		try_divide(+,-),
		div_parts(+, -),
		div_parts_helper(+,+,-),
		longest_side(r,-),
		shortest_side(r,-)
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

furniture_rel_pose_perceive(Furniture, PoseStampedList) :-
	% Get the PoseStamped of the Furniture
	object_pose(Furniture, [Frame, [X,Y,Z], Rotation]),
	object_shape_workaround(Furniture, _, ShapeTerm,_,_),
	longest_side(Furniture, LSize),
	shortest_side(Furniture, SSize),
	%dir_size('-y', ShapeTerm, Size),
	dir_size('-z', ShapeTerm, ZSize),
	ZNew is Z - ZSize,
	XNew is X - (SSize / 2.0) - 0.7, 
	YNew is Y - (LSize / 2.0),
	try_divide(LSize, Middle),
	build_pose_stamped_list(Middle, [Frame, [XNew,YNew,ZNew], Rotation], PoseStampedList).
	%append(,PoseStampedList,PoseStampedListe).

	build_pose_stamped_list([], _, []).
	build_pose_stamped_list([YN | Rest], [Frame, [X,Y,Z], Rotation], [PoseStamped | RestPoses]) :-
	% Build PoseStamped with different Y positions
	YNew is Y + YN, 
	PoseStamped = [Frame, [X,YNew,Z], Rotation],
	build_pose_stamped_list(Rest, [Frame, [X,Y,Z], Rotation], RestPoses).


	try_divide(Size, Midpoints) :-
		(   Size > 0.8 
		->	div_parts_helper(Size, [0.4], Midpoints)
		;   div_parts(Size, Midpoints)
		).	
	div_parts(Size, [Middle]) :-
		Size > 0, 
		Middle is Size / 2.0,
		writeln("Dividing using div_parts"),
		writeln("Middle: " + Middle).
	
	div_parts_helper(Size, [Middle | Rest], [Middle | Midpoints]) :-
		Middle < Size,
		NextMiddle is Middle + 0.7,
		writeln("Dividing using div_parts_helper"),
		writeln("Middle: " + Middle),
		div_parts_helper(Size, [NextMiddle | Rest], Midpoints).
	
	div_parts_helper(Size, [Middle | Rest], Midpoints) :-
		Middle >= Size,		
		writeln("Ending recursion"),
		(   Rest = [] ->
			Midpoints = [Size]
		;   Midpoints = Rest 
		).
	

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

%% longest_side(r Furniture, - Size)
%
% returns the longest side of a non rectangular furniture item 
% 	and in case of a rectangular also YSize  
longest_side(Furniture, Size):-
	object_shape_workaround(Furniture, _, ShapeTerm, _, _),
	dir_size('-x', ShapeTerm, XSize),
	dir_size('-y', ShapeTerm, YSize),
	( XSize >= YSize 
	-> Size = XSize
	; Size = YSize
	).

shortest_side(Furniture, Size):-
	object_shape_workaround(Furniture, _, ShapeTerm, _, _),
	dir_size('-x', ShapeTerm, XSize),
	dir_size('-y', ShapeTerm, YSize),
	( XSize < YSize 
	-> Size = XSize
	; Size = YSize
	).

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
