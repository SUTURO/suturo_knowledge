:- module(mocking,
    [
      create_object_on_surface/1
    ]).

:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2018/10/objects#', [keep(true)]).

:- rdf_meta
    create_object_on_surface(?).


create_objects_on_table :-
    create_banana_on_table,
    create_milk_on_table,
    create_coffee_on_table,
    create_unknown_on_table.
    %group_table_objects.

create_object_on_surface(Surface) :-
    surface_pose_in_map(Surface, Pose),
    rdf_urdf_link_collision(Surface, ShapeTerm, _),
    rdf_urdf_has_child(Joint, Surface),
    joint_abs_position(Joint,[JPosX,JPosY,JPosZ]),
    joint_abs_rotation(Joint,[Roll,Pitch,Yaw]),
    find_corners([JPosX,JPosY,_], [Roll,Pitch,Yaw], ShapeTerm, Corners),
    find_random_suitable_pos(Pos, Corners, JPosZ),
    euler_to_quaternion([Roll, Pitch, Yaw], [X,Y,Z,W]),
    Transform = ['map', _, Pos, [X,Y,Z,W]],
    create_object_at('http://www.semanticweb.org/suturo/ontologies/2018/10/objects#Banana',
        0.8, 
        Transform,
        0.05, 
        ObjectInstance, 
        [0.2, 0.075, 0.075], 
        box,
        0.8,
        [0.0,0.0,0.0,0.0],
        0.9),
    place_object(ObjectInstance).


% Expects the second argument to be the corners of a surface generated by find_corners/4
find_random_suitable_pos([X,Y,Z], [[X1,Y1],[X2,Y2],[X3,Y3],[X4,Y4]], Z) :-
    XList = [X1,X2,X3,X4],
    YList = [Y1,Y2,Y3,Y4],
    min_list(XList, XMin),
    max_list(XList, XMax),
    min_list(YList, YMin),
    max_list(YList, YMax),
    find_random_suitable_pos_(XMin, XMax, YMin, YMax, Pos, Z),
    Pos = [X,Y].

find_random_suitable_pos_(XMinInp, XMaxInp, YMinInp, YMaxInp, Pos, Z) :-
    XMin is XMinInp - 0.1,
    XMax is XMaxInp - 0.1,
    YMin is YMinInp - 0.1,
    YMax is YMaxInp - 0.1,
    random(XMin, XMax, X),
    random(YMin, YMax, Y),
    Transform = ['map', _, [X,Y,Z], [0.0,0.0,0.0,0.0]],
    (   hsr_existing_object_at(Transform, 0.01, _)
        -> find_random_suitable_pos_(XMinInp, XMaxInp, YMinInp, YMaxInp, Pos)
        ; Pos = [X,Y]
    ).