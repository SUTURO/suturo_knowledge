:- module(id_to_text,
    [
    objectid_to_text/2,
    surfaceid_to_text/2,
    roomid_to_text/2,
    colorclass_to_text/2
    ]).

:- rdf_meta
    objectid_to_text(r,?),
    surfaceid_to_text(r,?),
    roomid_to_text(r,?).

objectid_to_text(ObjID,Name):-
    (ask(has_type(ObjID, ObjClass))
    ->  (ask(triple(ObjClass, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName',_))
        -> ask(triple(ObjClass, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName', Name))
        ; (
            ros_warn("nlg.pl unable to deterimine name no TtSName"),
            split_string(ObjClass, "#","",L),
            nth0(1,L,Name)
        )
    )
    ; (
        ros_warn("nlg.pl unable to deterimine name not a Class"), 
        split_string(ObjID, "#","",L), 
        nth0(1,L,A),
        split_string(A, "_","",B), 
        nth0(0,B,Name)
      )
    ).


surfaceid_to_text(SurID,Name):-
    % TODO what is SurID?
    % like iai_kitchen/bookshelf_clone_0_floor_1_piece ?
    % like http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject_BWLUONCK ?
    % or
    % like bookshelf_clone_0_floor_1_piece // bed_table_center ?
    % depending use urdf_frame(http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject_BWLUONCK , bookshelf_clone_0_floor_1_piece)
    % this code expects bookshelf_clone_0_floor_1_piece

    split_string(SurID,"_","",Split),
    % Split e.g. ["bed", "table", "center"]
    ignore(nth1(Index,Split,"table")),
    ignore(nth1(Index,Split,"bookshelf")),
    ignore(nth1(Index,Split,"shelf")),
    ignore(nth1(Index,Split,"shefl")),
    ignore(nth1(Index,Split,"bucket")),
    (number(Index); % When we didnt get an Index
    length(Split,Index)), % we use the max index
    % Index e.g. 2
    findall(E, (nth1(I,Split,E), I =< Index), NameSplit),
    % NameSplit e.g. ["bed", "table"]
    atomic_list_concat(NameSplit, " ", NameWoThe),
    % NameWoThe e.g. bed table 
    atomic_concat('the ', NameWoThe, Name).
    % Name e.g. the bed table 


colorclass_to_text(ColorClass, ColorName):-
    split_string(ColorClass,"#","",Split),
    nth0(1,Split,ColorName).


roomid_to_text(RoomID,Name):-
    Name  = RoomID.
