:- module(clean_table,
	  [
	      temporary_storage_surface/1,
	      temporary_storage_pose/2,
	      stored_objects/1,
	      source_pose/3,
	      default_surface/2,
	      get_current_or_default_surface/2,
	      get_current_or_default_surfaces/2
	  ]).

:- use_module(library('beliefstate')).
:- use_module(library('locations/spatial_comp'),
	[
            surface_dimensions/4
	]).

:- use_module(library('model/environment/surfaces'),
	   [
	       has_table_shape/1
	   ]).

:- rdf_meta
   temporary_storage_surface(?),
   temporary_storage_pose(-, ?).

:- dynamic temporary_storage_group/1.

%% temporary_storage_surface(?Surface) is nondet.
%
% Currently this just returns the Surface of the long_table, but this may be changed to something dynamic in the future
temporary_storage_surface(Surface) :-
    has_urdf_name(Surface,"long_table:table:table_center").

%% temporary_storage_pose(+Instance, ?Pose) is nondet.
%
% Look where on temporary_storage_surface the Object instance should go.
% It also adds the Object to a group and believes the Object has been placed there for all future calculations.
%
% @param Instance The Object instance.
% @param Pose The Pose as [[x, y, z], [qx, qy, qz, qw]] relative to the map frame.
temporary_storage_pose(Instance, [Translation, Rotation]) :-
    temporary_storage_surface(Surface),
    temporary_storage_pose_(Instance, [Translation, Rotation], Surface).

%% temporary_storage_pose_(+Instance, ?Pose, +Surface) is nondet.
%
% This Predicate calculates where on the surface the Instance should go.
% It also adds the Object to a group and believes the Object has been placed there for all future calculations.
%
% @param Instance The Object instance.
% @param Pose The Pose where the Object should be placed.
% @param Surface The temporary_storage_surface where the Object should be placed on.
temporary_storage_pose_(Instance, [Translation, Rotation], Surface) :-
    %% The logging here is done in case there is an error in this method and you want to quickly look at what failed.
    format(string(Log1), "temporary_storage_pose_('~w', [Translation, Rotation], '~w')", [Instance, Surface]),
    ros_info(Log1),

    %% This code is copied mostly from object_goal_pose in gripper/placing.pl
    has_table_shape(Surface),
    has_urdf_name(Surface, SurfaceLink),
    %% Note: the SurfaceLink is in the center of the surface

    min_space_between_objects(MinSpace),
    object_dimensions(Instance, ObjectWidth, ObjectDepth, ObjectHeight),
    all_groups_on_tablelike_surface(Surface, Groups),
    surface_dimensions(Surface, SurfaceWidth, SurfaceDepth, SurfaceHeight),
    length(Groups, NGroups),
    format(string(Log2), "temporary_storage_pose_/3 variables. Groups: ~w, NGroups:~w", [Groups, NGroups]),
    ros_info(Log2),
    
    %      to the front   and space away and to the center of the object
    RefX is (-SurfaceWidth/2) + MinSpace + ObjectDepth/2,
    
    RefZ is ObjectHeight/2,


    (NGroups = 0
    ->
	(
	    %%      to the left and space away and to the center of the object
            RefY is SurfaceDepth/2 - MinSpace - ObjectWidth/2,
            tf_transform_pose(SurfaceLink, 'map', pose([RefX, RefY, RefZ], [0, 0, 0, 1]), pose(Translation, Rotation)),

            tell(has_type(NewGroup, hsr_objects:'Group')),
            tell(triple(Instance, hsr_objects:'inGroup', NewGroup)),
	    assertz(temporary_storage_group(NewGroup))
        );
     (NGroups = 1 ->
          (
              [Group] = Groups,

	      %% group_position_on_surface returns the hind left corner, but we need the front right
	      group_position_on_surface(Group, Surface, [GroupX, GroupY, _]),
              group_dimensions(Group, GroupDepth, GroupWidth),
	      
	      format(string(Log3), "group info: XY= ~w, ~w; DW= ~w ~w", [GroupX, GroupY, GroupDepth, GroupWidth]),
	      ros_info(Log3),

	      %% start at the back of the group, go to the front, and go back to the center of the object.
	      %% this way all object align their start at the local x coordinate

	      % Don't rely on the rest of the group, just get the number directly from the Surface before the branch.
	      % It looks like i don't understand groups fully.
	      
	      % % 0.17 as temp fix for GroupDepth being way too high.
	      % RefX is GroupX - 0.17 + ObjectDepth/2,

	      %% start at the left, go to the right, add the spacer, go to the center of the new object
	      RefY is GroupY - GroupWidth - MinSpace - ObjectWidth/2,
              tf_transform_pose(SurfaceLink, 'map', pose([RefX, RefY, RefZ], [0, 0, 0, 1]), pose(Translation, Rotation)),
	      tell(triple(Instance, hsr_objects:'inGroup', Group))
          );
      %% Idk how groups are supposed to work and i don't have the time to try it now.
      %% Because of that i won't implement handling more than one group on a surface
      (ros_info("Ngroups Fail"),fail()))),
    !.

%% stored_objects(?Ìnstances) is nondet.
%
% Find all objects in the group that is used for temporary_storage_pose.
%
% @param Instances a list containing all found Objects.
stored_objects(Instances) :-
    temporary_storage_group(Group),
    findall(Instance, triple(Instance, hsr_objects:'inGroup', Group), Instances).

%% source_pose source_pose(?ObjID, ?Frame, ?Pose) is nondet.
source_pose(ObjID, Frame, [[X, Y, Z], [RX, RY, RZ, RW]]) :-
    triple(ObjID, suturo:'start_pose_frame', Frame),
    
    triple(ObjID, suturo:'start_pose_x', X),
    triple(ObjID, suturo:'start_pose_y', Y),
    triple(ObjID, suturo:'start_pose_z', Z),

    triple(ObjID, suturo:'start_pose_rx', RX),
    triple(ObjID, suturo:'start_pose_ry', RY),
    triple(ObjID, suturo:'start_pose_rz', RZ),
    triple(ObjID, suturo:'start_pose_rw', RW).

%% default_surface(?Class, ?Surface) is nondet.
%
% @tbd nicht mit predicates sondern in die owl dateien
:- dynamic default_surface/2.

%% get_current_or_default_surface(+Type, ?Surface) is nondet.
%
% Get the surface where an Object of the Type is or a default surface for this Type using default_surface/2.
% Currently does not work correctly, needs to be debugged.
get_current_or_default_surface(Type, Surface) :-
    has_type(Object, Type),
    object_supported_by_surface(Object, Surface).

get_current_or_default_surface(Type, Surface) :-
    default_surface(Type, Surface).

%% get_current_or_default_surfaces(+Type, ?Surfaces) is nondet.
%
% This is just a findall on get_current_or_default_surface/2.
get_current_or_default_surfaces(Type, Surfaces) :-
    findall(Surface, get_current_or_default_surface(Type, Surface), Surfaces).
