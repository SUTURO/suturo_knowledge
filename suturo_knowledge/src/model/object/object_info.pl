%% The object info module contains predicates that provide information about the objects and their role in the world.
:- module(object_info,
	  [
        object_rel_pose/3,
        object_pose/2
	  ]).

:- use_module(library('ros/tf/tf'),
	      % actually uses tf:tf_get_pose, but that is not exported by tf
	      []).

%% object_pose(+Object, -PoseStamped) is semidet.
%
% Get the pose of an object.
object_pose(Object, PoseStamped) :-
    tf:tf_get_pose(Object, PoseStamped).

%% object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to the object based on the type of relation.
%
% @param Object The Object to which the position is relative to.
% @param Type The type of relation. It can be "perceive", "interact" or "destination".
% @param PoseStamped The position relative to the Object.
%
object_rel_pose(Object, Type, PoseStamped) :-
    % call the appropriate predicate based on the type of the object
    (has_type(Object, soma:'DesignedComponent')
     ->	component_rel_pose(Object, Type, PoseStamped)
    ; has_type(Object, soma:'DesignedContainer')
     ->	container_rel_pose(Object, Type, PoseStamped)
    ; has_type(Object, soma:'DesignedFurniture')
     ->	furniture_rel_pose(Object, Type, PoseStamped)
    ; ros_error('Error: Unknown object class for object_rel_pose: object ~w.', [Object]),
      false
    ).
