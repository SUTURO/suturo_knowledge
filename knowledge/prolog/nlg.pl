:- module(nlg,
    [
    object_tts/2,
    surface_tts/2,
    room_tts/2
    ]).

:- rdf_meta
    object_tts(r,?),
    surface_tts(r,?),
    room_tts(r,?).

object_tts(ObjID,Name):-
    (rdfs_individual_of(ObjID, ObjClass)
    -> ( rdf_has(ObjClass, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName',_)
        -> rdf_has(ObjClass, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName', Name)
        ; ros_error("nlg.pl unable to deterimine name"), Name = ObjID
        )
    ; ros_error("nlg.pl unable to deterimine name"), Name = ObjID
    ).

%TODO
surface_tts(SurID,Name):-
    Name  = SurID.

%TODO
room_tts(RoomID,Name):-
    Name  = RoomID.
