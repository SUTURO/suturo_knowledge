% furniture information
:- module(furniture_info,
	[
		furniture_rel_pose(r,+,-),
        has_robocup_name(r,?),
		try_divide(+,-),
		div_parts(+, -),
		div_parts_helper(+,+,-),
		longest_side(r,-),
		shortest_side(r,-),
		get_name_handle(r, -),
		which_handle(r, -),
		has_door(?),
		which_door(?, ?),
		has_handle(r),
		storable_at(r, -),
		open(?,r),
		find_triple(r, -),
		add_milk(?)
	]).

:- ros_warn("SHOULD HAVE DOOR").

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

% --- FallSchool --- define some facts
% furniture_handle(FurnitureObject, Handle).
%furniture_handle(soma:'Refrigerator', 'handle_cab3_door_top').
%furniture_handle('Refrigerator', 'handle_cab3_door_top').
%furniture_handle('Fridge', 'handle_cab3_door_top').

%get_name_handle(Furniture, HandleLinkName):-
%	furniture_handle(Furniture, HandleLinkName).

% where is the Milk Located?


% milk is perishable

has_quality(Entity, suturo:'Perishable') :-
	instance_of(Entity,suturo:'Milk').

% all fridges are containers

has_type(Entity, soma:'DesignedContainer') :-
    has_type(Entity, soma:'Refrigerator').

% where to store something?

% all fridges are containers for perishable items
storable_at(Item, Location) :-
    has_quality(Item, suturo:'Perishable'),
	instance_of(Location, soma:'Refrigerator').

% doors and cups have handles

has_handle(Entity) :-
    instance_of(Entity, soma:'Door').

has_handle(Entity) :-
    instance_of(Entity, soma:'Cup').

% Descendant link

has_descendant(Ancestor, Descendant) :-
    urdf_link_child_joints(arena, Ancestor, Children),
	member(Joint, Children),
	urdf_joint_child_link(arena, Joint, Descendant).

has_descendant(Ancestor, Descendant) :-
    urdf_link_child_joints(arena, Ancestor, Children),
	member(Joint, Children),
	urdf_joint_child_link(arena, Joint, Child),
	has_descendant(Child, Descendant).

% which link is the handle of something?

which_handle(Object, Handle) :-
    has_handle(Object),
	has_urdf_name(Object, URDFName),
	has_descendant(URDFName, URDFHandle),
	has_urdf_name(Handle, URDFHandle),
	instance_of(Handle, soma:'DesignedHandle').

% all fridges have doors

has_door(Entity) :-
    ros_info("STUFF IS GOING DOWN"),
    ros_info("HasDoor entity: ~w", [Entity]),
    instance_of(Entity, soma:'Refrigerator'),
    ros_info("STUFF KEEPS GOING DOWN").

% which link is the door of something?

which_door(Object, Door) :-
    has_door(Object),
	has_urdf_name(Object, URDFName),
	%%urdf_link_child_joints(arena, URDFName, Children),
	%%member(Joint, Children),
	%%urdf_joint_child_link(arena, Joint, URDFDoor),
	has_descendant(URDFName, URDFDoor),
	has_urdf_name(Door, URDFDoor),
	instance_of(Door, soma:'Door'),
    urdf_link_child_joints(arena, URDFDoor, Children),
	member(Joint, Children),
	urdf_joint_child_link(arena, Joint, URDFHandle),
	has_urdf_name(Handle, URDFHandle),
	instance_of(Handle, soma:'DesignedHandle').



%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% is a fridge currently opened or closed?

%joint_angle('door joint', 0).
%has_type('alice', soma:'Refrigerator').
%has_open_angle('alice', 50).

open(Door, Angle) :-
    has_urdf_name(Door, URDFDoor),
	urdf_link_parent_joint(arena, URDFDoor, Joint),
	urdf_joint_hard_limits(arena, Joint, [_, Max], _, _),
	Dif is Angle - Max,
	Dif > -0.2 .

% Add Milk
add_milk(Milk):-
	 kb_project([
                new_iri(Milk, suturo:'Milk'), 
				is_individual(Milk),
				instance_of(Milk, suturo:'Milk')]).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%has_type('alice', soma:'Refrigerator').
%has_type('bob', soma:'Car').

%%    has_type(Entity, soma:'Refrigerator').


%>> has_type(X, soma:'Car')?
%(X . 'bob'))

%>> has_type('alice', Y)?
%((Y. soma:'Refrigerator'), ( Y . soma:'DesignedContainer'))

%>> has_type(X, soma:'DesignedContainer')
%((X . 'alice'))
find_triple(Q, T) :- 
	(atom(Q) -> atom_string(Q, Qstr) ; Qstr is Q), string_lower(Qstr, Qlwr), % attempt to bring the query to a canonical form: lowercase string
  	triple(S, P, O), % for all triples in the knowledge base
  	(atom(S) -> atom_string(S, Sstr) ; Sstr is S), string_lower(Sstr, Slwr), % attempt to bring the triple elements into a canonical form: lowercase string
  	(atom(P) -> atom_string(P, Pstr) ; Pstr is P), string_lower(Pstr, Plwr),
  	(atom(O) -> atom_string(O, Ostr) ; Ostr is O), string_lower(Ostr, Olwr),
  	(                                                                        % attempt to find the query in some triple element
    	sub_string(Slwr, _, _, _, Qlwr);
    	sub_string(Plwr, _, _, _, Qlwr);
    	sub_string(Olwr, _, _, _, Qlwr)
  	),
  	=(T, [S, P, O]).