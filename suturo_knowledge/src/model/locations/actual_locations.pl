:-module(actual_locations,
    [
        object_locations/2,
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
object_locations(Object, Location):-
    triple(Object, suturo:hasDataSource, perception),
    kb_call(is_at(Object,[map,Location,_])).


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
    kb_call(is_at(base_footprint,[map,Location,_])).

    
