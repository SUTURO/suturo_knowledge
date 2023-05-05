:-module(actual_locations,
    [
        object_locations/1,
        table_locations/1,
        shelf_locations/1,
        drawer_locations/1,
        robot_location/1
    ]
    ).

 :- use_module(library('model/object/object_info'),
    [
        object_pose/2
        
    ]).
 

%get actual object locations

%create_object, object_pose
object_locations(Location):-
    fail.
    %create_object(Object, Type, PoseStamped).
    %object_pose(Object, PoseStamped).


%get actual furniture location
table_locations(Location) :-
    is_table(Table),
    object_pose(Table,Location).

shelf_locations(Location) :-
    is_shelf(Shelf),
    object_pose(Shelf, Location).

drawer_locations(Location) :-
    is_drawer(Drawer),
    object_pose(Drawer, Location).

%get actual surface location

%get actual robot location
robot_location(Location):-
    object_pose(base_footprint, Location).
    
