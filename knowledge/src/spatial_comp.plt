:- begin_tests('spatial_comp').


:- use_module(library('test')).
:- use_module(library('urdf')).


:- include('spatial_comp').

:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


test(surface_pose_in_map_with_surface_table) :-
	surface_pose_in_map('table_center', [Position, Rotation]),
	tf_lookup_transform('map', 'table_front_edge_center', pose(ExpPosition, ExpRotation)),
	assert_true(Position == ExpPosition),
	assert_true(Rotation == ExpRotation).

test(surface_pose_in_map_with_surface_shelf) :-
	surface_pose_in_map('bookshelf_floor_1_piece', [Position, Rotation]),
	tf_lookup_transform('map', 'bookshelf_floor_1_piece', pose(ExpPosition, ExpRotation)),
	assert_true(Position == ExpPosition),
	assert_true(Rotation == ExpRotation).

test(surface_pose_in_map_with_surface_bucket) :-
	surface_pose_in_map('bucket_center', [Position, Rotation]),
	tf_lookup_transform('map', 'bucket_surface_center', pose(ExpPosition, ExpRotation)),
	assert_true(Position == ExpPosition),
	assert_true(Rotation == ExpRotation).

test(fail) :-
	fail.


:- end_tests(spatial_comp).