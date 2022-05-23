:- module(drawers,
	  [
	      store_drawer_knob/3,
	      drawer_knob/3
	  ]).
% Module for drawer stuff that doesn't fit into furnitures or surfaces

:- rdf_db:rdf_register_ns(hsr_rooms, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#', [keep(true)]).

store_drawer_knob(ObjID, [Position, Rotation], box(Width, Depth, Height)) :-
    tell(has_type(ObjID, hsr_rooms:drawer_knob)),
    tell(triple(ObjID, hsr_rooms:hasPosition, term(Position))),
    tell(triple(ObjID, hsr_rooms:hasRotation, term(Rotation))),
    tell(triple(ShapeRegion, hsr_rooms:hasWidth, Width)),
    tell(triple(ShapeRegion, hsr_rooms:hasDepth, Depth)),
    tell(triple(ShapeRegion, hsr_rooms:hasHeight, Height)).

drawer_knob(ObjID, [Position, Rotation], box(Width, Depth, Height)) :-
    has_type(ObjID, hsr_rooms:drawer_knob),
    triple(ObjID, hsr_rooms:hasPosition, term(Position)),
    triple(ObjID, hsr_rooms:hasRotation, term(Rotation)),
    triple(ShapeRegion, hsr_rooms:hasWidth, Width),
    triple(ShapeRegion, hsr_rooms:hasDepth, Depth),
    triple(ShapeRegion, hsr_rooms:hasHeight, Height).
