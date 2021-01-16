:- module(export,
    [
      % Roles
      make_all_tables_source/0,
      make_all_shelves_target/0,
      make_all_buckets_target/0,
      make_ground_source/0,
      % Find Objects
      all_objects_on_tables/1,
      all_objects_in_whole_shelf/1,
      % Place Objects
      object_goal_surface/2,
      object_goal_surface/3,
      object_goal_pose_offset/3,
      % Pick up Object
      next_object/1,
      % URDF
      surface_frame/2,
      pose_of_shelves/1,
      % Find Surfaces
      table_surfaces/1,
      % Beliefstate
      forget_objects_on_surface/1
      % Mocking
      % create_object_on_surface/1

    ]).

%%% ROLES %%%

make_ground_source:-
    make_all_surface_type_role(ground, source).

make_all_shelves_target:-
    make_all_surface_type_role(shelf, target).

make_all_tables_source:-
    make_all_surface_type_role(table, source).

make_all_buckets_target:-
    make_all_surface_type_role(bucket, target).

%%% FIND OBJECTS %%%

all_objects_on_tables(Objects) :-
    all_objects_on_tables_(Objects).

all_objects_in_whole_shelf(Objects) :-
    all_objects_in_whole_shelf_(Objects).

%%% PLACE OBJECTS %%%

object_goal_surface(Object, Surface) :-
    object_goal_surface_(Object, Surface, _, _).

object_goal_surface(Object, Surface, Context) :-
    object_goal_surface_(Object, Surface, Context, _).

object_goal_pose_offset(Object, Pose, Context) :-
    object_goal_pose_offset_(Object, Pose, Context).

%%% PICK UP OBJECT %%%

next_object(Object) :-
    next_object_(Object).

%%% BELIEFSTATE %%%

forget_objects_on_surface(Surface) :-
    forget_objects_on_surface_(Surface).

%%% URDF %%%

surface_frame(Surface, Frame) :-
    surface_frame_(Surface, Frame).
