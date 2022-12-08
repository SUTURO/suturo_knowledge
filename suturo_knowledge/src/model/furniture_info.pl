%furniture informations
:- module(furniture_info,
	  [
	      %get_drawer_pose/1,
	      get_table_pose/1
	  ]).

%%
%
%
%get_drawer_pose(Pose) :-
    

get_table_pose(Pose) :-
    tf_lookup_transform('map', 'iai_kitchen/tall_table:table:table_front_edge_center', pose(Position, Rotation)),
    Pose = [map, Position, Rotation].