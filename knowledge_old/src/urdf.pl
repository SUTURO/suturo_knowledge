:- module(urdf,
    [
        load_surfaces_from_param/1,
        surface_frame_add_prefix_/2,
        urdf_frame_add_prefix_/2,
        object_tf_frame/2,
        urdf_tf_frame/2,
        has_urdf_name/2,
        get_urdf_id/1,
        get_urdf_origin/1
    ]).


%:- rdf_meta().
    

/**
* is called as inital_goal in the launch file
*/
load_surfaces_from_param(Param):-
    ros_param_get_string(Param,S), % S is the urdf file (a xml file) as a string
    get_urdf_id(URDF),
    urdf_load_xml(URDF,S),
    get_urdf_origin(Origin),
    urdf_set_pose_to_origin(URDF,Origin),
    urdf_link_names(URDF,Links).



object_tf_frame(Object, Frame) :-
    ( sub_string(Object,_,_,_,"#")
    -> split_string(Object, "#", "", [_, Frame])
    ;
    Frame = Object
    ).

urdf_tf_frame(UrdfInstance, Frame) :-
    has_urdf_name(UrdfInstance, UrdfName),
    urdf_frame_add_prefix_(UrdfName, Frame).

    
surface_frame_add_prefix_(SurfaceName, Surface_with_Prefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, SurfaceName, Surface_with_Prefix).

urdf_frame_add_prefix_(UrdfName, UrdfNameWithPrefix) :-
    urdf_surface_prefix(Prefix),
    atom_concat(Prefix, UrdfName, UrdfNameWithPrefix).


has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).

urdf_surface_prefix(Prefix) :-
    Prefix = 'iai_kitchen/'.

get_urdf_id(URDF) :-
    URDF = arena.

get_urdf_origin(Origin) :-
    Origin = map.


    

