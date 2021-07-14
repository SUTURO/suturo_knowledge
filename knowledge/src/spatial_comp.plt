:- begin_tests('spatial_comp').


:- use_module(library('test')).
:- use_module(library('urdf')).


:- use_module('spatial_comp.pl').


test(setup) :-
    setup_suturo_rooms_test_env,
    setup_suturo_test_objects,
    setup_suturo_test_rooms,
    setup_suturo_test_furnitures.


test(is_legal_obj_position_when_position_is_legal) :-
    tf_lookup_transform(map, 'iai_kitchen/bed#bed#table_center', Pose),
    writeln(Pose),
    is_legal_obj_position([2.5, 4.0, 0.619873]).


test(object_in_room) :-
	get_suturo_test_objects([Bowl1, _, _, _]),
	tell(has_type(Location, soma:'Location')),
	tell(triple(Bowl1, dul:'hasLocation', Location)),
	in_room(Bowl1, Room),
	has_type(ExpRoom, hsr_rooms:'LivingRoom'),
	assert_equals(Room, ExpRoom).


test(object_on_furniture) :-
    get_suturo_test_objects([Bowl1, _, _, _]),
	tell(has_type(Location, soma:'Location')),
	tell(triple(Bowl1, dul:'hasLocation', Location)),
	on_furniture(Bowl1, Furniture),
	triple(ExpFurniture, urdf:'hasURDFName', 'couch_table#table#table_front_edge_center'),
	assert_equals(Furniture, ExpFurniture).


%test(surface_pose_in_map_with_surface_table) :-
%	surface_pose_in_map('table_center', [Position, Rotation]),
%	tf_lookup_transform('map', 'table_front_edge_center', pose(ExpPosition, ExpRotation)),
%	assert_true(Position == ExpPosition),
%	assert_true(Rotation == ExpRotation).

%test(surface_pose_in_map_with_surface_shelf) :-
%	surface_pose_in_map('bookshelf_floor_1_piece', [Position, Rotation]),
%	tf_lookup_transform('map', 'bookshelf_floor_1_piece', pose(ExpPosition, ExpRotation)),
%	assert_true(Position == ExpPosition),
%	assert_true(Rotation == ExpRotation).

%test(surface_pose_in_map_with_surface_bucket) :-
%	surface_pose_in_map('bucket_center', [Position, Rotation]),
%	tf_lookup_transform('map', 'bucket_surface_center', pose(ExpPosition, ExpRotation)),
%	assert_true(Position == ExpPosition),
%	assert_true(Rotation == ExpRotation).
	

:- end_tests(spatial_comp).