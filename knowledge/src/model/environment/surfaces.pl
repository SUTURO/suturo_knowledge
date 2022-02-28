    
:- module(surfaces,
    [   create_surface/3,
        surfaces_not_visited/1,
        set_surface_visited/1,
        set_surface_not_visited/1,
        all_surfaces/1,
        is_surface/1,  
        get_perception_surface_region/2,
        has_table_shape/1,
        has_shelf_shape/1,
        has_bucket_shape/1
    ]).


:- tripledb_load(
	'package://knowledge/owl/rooms.owl',
	[ namespace(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#')
	]).


%% is_surface(?Surface) is nondet.
%
% True if Surface is an instance of soma:'Surface'.
%
% @param Surface A surface IRI.
%
is_surface(Surface) :-
    has_type(Surface, soma:'Surface').


%% all_surfaces(?Surfaces) is det.
%
% Returns all instances of type soma:'Surface'.
%
% @param Surfaces A list of surface instances
%
all_surfaces(Surfaces) :-
    findall(Surface, is_surface(Surface), Surfaces).


%% has_table_shape(?Surface) is nondet
%
% True if Surface has a shape of type hsr_rooms:'TableShape'
%
% @param Surface A surface IRI
%
has_table_shape(Surface) :-
    is_surface_of(Surface, Furniture),
    triple(Furniture, soma:'hasShape', hsr_rooms:'TableShape'),
    not has_bucket_shape(Surface).


%% has_shelf_shape(?Surface) is nondet
%
% True if Surface has a shape of type hsr_rooms:'ShelfShape'
%
% @param Surface A surface IRI
%
has_shelf_shape(Surface) :-
    is_surface_of(Surface, Furniture),
    triple(Furniture, soma:'hasShape', hsr_rooms:'ShelfShape').


%% has_bucket_shape(?Surface) is nondet
%
% True if Surface is surface of a furniture of type hsr_rooms:'Deposit'
%
% @param Surface A surface IRI
%
has_bucket_shape(Surface) :-
    is_surface_of(Surface, Furniture),
    has_type(Furniture, hsr_rooms:'Deposit').


%% is_surface_of(?Surface, ?Furniture) is nondet
%
% True if Surface is a surface of Furniture
%
% @param Surface a surface IRI
% @param Furniture a furniture IRI
%
is_surface_of(Surface, Furniture) :-
    triple(Furniture, hsr_rooms:'hasSurface', Surface).

    
%% surfaces_not_visited(?Surfaces) is det.
%
% Returns all surfaces with hsr_rooms:'VisitState' false
%
% @param Surfaces A list of surface instances
%
surfaces_not_visited(Surfaces) :-
    findall(Surface, ( is_surface(Surface), not visited(Surface)), Surfaces).


%% set_surface_visited(?Surface)
%
% Sets hsr_rooms:'VisitState' of surface to true
%
% @param Surface A surface IRI
%
set_surface_visited(Surface) :-
    update_visit_state(Surface, true).


%% set_surface_not_visited(?Surface)
%
% Sets hsr_rooms:'VisitState' of surface to false
%
% @param Surface A surface IRI
%
set_surface_not_visited(Surface) :-
    update_visit_state(Surface, false).


%% visited(?Surface) is nondet.
%
% True if surface has a visit state value of false
%
% @param Surface A surface IRI
%
visited(Surface) :-
    is_surface(Surface),
    triple(Surface, hsr_rooms:'hasVisitState', VisitState),
    triple(VisitState, hsr_rooms:'visited', Visited),
    not Visited == 0.


%% init_visit_state(?Surface)
%
% Sets hsr_rooms:'VisitState' of surface to initial value of false
%
% @param Surface A surface IRI
%
init_visit_state(Surface) :-
    tell(has_type(VisitState, hsr_rooms:'VisitState')),
    tell(triple(Surface, hsr_rooms:'hasVisitState', VisitState)),
    tell(triple(VisitState, hsr_rooms:'visited', true)).


%% update_visit_state(?Surface, ?State) is nondet.
%
% Sets hsr_rooms:'VisitState' of surface to value of State
%
% @param Surface A surface IRI, State A boolean of value true or false
%
update_visit_state(Surface, State) :-
    is_surface(Surface),
    triple(Surface, hsr_rooms:'hasVisitState', VisitState),
    forall(triple(VisitState, hsr_rooms:'visited', _), tripledb_forget(VisitState, hsr_rooms:'visited', _)),
    tell(triple(VisitState, hsr_rooms:'visited', State)).


%% get_perception_surface_region(Surface, ?PerceptionName)
%
% Returns the region name of the given surface instance
%
% @param Surface A surface IRI, PerceptionName region name as string
%
%% get_perception_surface_region(Surface, PerceptionName):-
%%     has_shelf_shape(Surface),
%%     has_urdf_name(Surface,Name),
%%     split_string(Surface, ":","",SurfaceSplit),
%%     nth0(0,SurfaceSplit,Name),sub_atom(Surface, _, 1, 0, Number),
%%     string_concat(Name,"_floor_",Temp),
%%     string_concat(Temp,Number,PerceptionName),!.


%% get_perception_surface_region(Surface, ?PerceptionName)
%
% Returns the region name of the given surface instance
%
% @param Surface A surface IRI, PerceptionName region name as string
%
get_perception_surface_region(Surface, PerceptionName):-
    has_urdf_name(Surface,Name),
    split_string(Name, ":","",SurfaceSplit),
    (has_shelf_shape(Surface),Index = 2;
     not(has_shelf_shape(Surface)),Index = 0),
    nth0(Index,SurfaceSplit,PerceptionName).



%% create_surface(Shape, Link, ?Surface)
%
% Creates an instance of type soma:'Surface' of given shape
%
% @param Shape String, Link Surafce URDF Link, Surface A surface IRI
%
create_surface(Shape, Link, Surface) :-
    sub_string(Shape,_,_,_,"table"),
    tell(has_type(Surface, hsr_rooms:'Tabletop')),
    tell(has_urdf_name(Surface, Link)),
    init_visit_state(Surface).


%% create_surface(Shape, Link, ?Surface)
%
% Creates an instance of type soma:'Surface' of given shape
%
% @param Shape String, Link Surafce URDF Link, Surface A surface IRI
%
create_surface(Shape, Link, Surface) :-
    supported_surface(Link),
    sub_string(Shape,_,_,_,"shelf"),
    tell(has_type(Surface, hsr_rooms:'Shelffloor')),
    tell(has_urdf_name(Surface, Link)),
    init_visit_state(Surface).


%% create_surface(Shape, Link, ?Surface)
%
% Creates an instance of type soma:'Surface' of given shape
%
% @param Shape String, Link Surafce URDF Link, Surface A surface IRI
%
create_surface(Shape, Link, Surface) :-
    supported_surface(Link),
    sub_string(Shape,_,_,_,"bucket"),
    tell(has_type(Surface, hsr_rooms:'BucketOpening')),
    tell(has_urdf_name(Surface, Link)),
    init_visit_state(Surface).


%% supported_surface(?SurfaceLink)
%
% True if surface link is big enough
%
% @param SurfaceLink URDF Link as string
%
supported_surface(SurfaceLink):-
    get_urdf_id(URDF),
    urdf_link_collision_shape(URDF,SurfaceLink,ShapeTerm,_),
    surface_big_enough(ShapeTerm).


surface_big_enough(box(X, Y, _)):- %TODO Support other shapes, has not been tested yet.
    square_big_enough(X,Y).


square_big_enough(X,Y):- %TODO Support other shapes
    Size = X * Y,
    (  Size >= 0.09 , X > 0.2, Y > 0.2
    -> true
    ; fail
    ).
