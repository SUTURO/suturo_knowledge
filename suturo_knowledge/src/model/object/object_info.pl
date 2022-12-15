%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
        object_rel_pose/3,
        object_dest_pose/2
	  ]).

%% object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to the object based on the type of relation.
%
% @param Object The Object to which the position is relative to.
% @param Type The type of relation. It can be "perceive" or "interact".
% @param PoseStamped The position relative to the Object.
%
object_rel_pose(Object, Type, PoseStamped) :-
    % if the object class is "DesignedFurniture", call the furniture_rel_pose/3 predicate
    (is_furniture(Object)
     ->	furniture_rel_pose(Object, Type, PoseStamped)
    ; ros_error('Error: Unknown object class of object ~w.', [Object])).

% TODO: use parentclass "DesignedFurniture"
is_furniture(Object) :-
    has_type(Object, soma:'Table'); 
    has_type(Object, soma:'Shelf');
    has_type(Object, soma:'Drawer').

%% object_dest_pose(+Object, -PoseStamped) is semidet.
%  
%  Returns the destination pose of the object.
%  The destination pose is the pose where the object should be placed.
%
%  @param Object The object for which the relative pose is requested.
%  @param Type The type of the pose.
%  @param PoseStamped The destination pose of the object.
%
object_dest_pose(Object, PoseStamped) :-
    % TODO get the destination pose of the object dynamically by reasoning
    % cereal_box_dest_pose(PoseStamped).
    (has_type(Object, soma:'CerealBox') 
     -> cereal_box_dest_pose(PoseStamped)
     ; ros_error('Unknown destination pose for object ~w of type ~w')).

:- trace(object_dest_pose).

cereal_box_dest_pose(PoseStamped) :-
    has_urdf_name(Destination, 'shelf:shelf:shelf_base_center'),
    object_pose(Destination, [Frame, [X,Y,Z], Rotation]),
    YNew is Y - 0.50,
    PoseStamped = [Frame, [X,YNew,Z], Rotation].