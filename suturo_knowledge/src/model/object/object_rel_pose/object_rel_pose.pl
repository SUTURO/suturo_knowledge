:- module(object_rel_pose,
	  [
	      object_rel_pose(r,+,-),
              object_rel_pose(r,+,+,-)
	  ]).

:- use_module(object_perceive_pose).
:- use_module(object_place_pose).
:- use_module(object_destination_pose, [object_destination_pose/3]).

%% object_rel_pose(+Object, +Type, -PoseStamped) is semidet.
%
% Gets an (optimal) position relative to the object based on the type of relation.
%
% @param Object The Object to which the position is relative to.
% @param Type The type of relation. It can be "perceive", "interact" or "destination".
% @param PoseStamped The position relative to the Object.
%
object_rel_pose(Object, Type, PoseStamped) :-
    object_rel_pose(Object, Type, [], PoseStamped).

:- predicate_options(object_rel_pose/4, 3,
		     [ pass_to(object_place_pose/3, 2),
		       pass_to(object_perceive_pose/3, 2)
		     ]).

%% object_rel_pose(+Object, +Type, +Options, -PoseStamped) is semidet.
%
% See object_rel_pose/3 for basic information.
% This predicate additionally accepts a list of options that might change the behavior of ceratin types.
%

object_rel_pose(Object, perceive, Options, PoseStamped) :-
    object_perceive_pose(Object, Options, PoseStamped), !.

object_rel_pose(Object, place, Options, PoseStamped) :-
    object_place_pose(Object, Options, PoseStamped), !.

object_rel_pose(Object, destination, Options, PoseStamped) :-
    object_destination_pose(Object, Options, PoseStamped), !.

object_rel_pose(Object, Type, _Options, PoseStamped) :-
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
