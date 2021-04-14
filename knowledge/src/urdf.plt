:- begin_tests(urdf).

:- use_module(library('test')).
:- use_module(library('surfaces')).

:- include(library('urdf')).

:- setup_suturo_test_env.
:- setup_suturo_test_surfaces.
:- setup_suturo_test_objects.


test(surface_tf_frame_with_surface_table) :-
    surface_tf_frame('table_center', Frame),
    assert_equals(Frame, 'table_front_edge_center').

test(surface_tf_frame_with_surface_bucket) :-
    surface_tf_frame('bucket_center', Frame),
    assert_equals(Frame, 'bucket_surface_center').

test(surface_tf_frame_with_surface_shelf) :-
    surface_tf_frame('bookshelf_floor_0_piece', Frame),
    assert_equals(Frame, 'bookshelf_floor_0_piece').

test(get_surface_id_by_name_when_surface_is_registered) :-
    get_surface_id_by_name('table_center', Id),
    assert_equals(Id, 'table_center').

test(get_surface_id_by_name_when_surface_not_registered) :-
    get_surface_id_by_name('hallo', Id),
    writeln(Id).


test(fail) :-
    fail.

:- end_tests(urdf).
