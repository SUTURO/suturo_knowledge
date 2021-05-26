:- module(nlp,
    [
    matching_object_in_room_name/3,
    matching_object_in_room_id/3,
    matching_object_everywhere/2,
    all_obj_names/1,
    all_obj_names_with_id/1,
    matching_obj_classes/2,
    matching_object_in_list/3,
    matching_room_class/2,
    matching_room/2,
    all_room_names_with_id/1
    ]).

:- rdf_meta
    all_obj_names(r),
    all_obj_names_with_id(r),
    matching_classes(r,r).


matching_object_in_room_name(ObjectName, RoomName, Obj):-
    matching_room(RoomName,RoomId),
    matching_object_in_room_id(ObjectName, RoomId, Obj).


matching_object_in_room_id(ObjectName, RoomId, Obj):-
    objects_in_room(RoomId,Objects),
    matching_object_in_list(ObjectName, Objects, Obj).


matching_object_everywhere(Name,Obj):-
    hsr_existing_objects(Objects),
    matching_object_in_list(Name ,Objects, Obj).

%%%%%OBJECTS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


all_obj_names(Names):-
    all_obj_names_with_id(AllNames),findall(Name, member([_,Name],AllNames),Names).

all_obj_names_with_id(AllNames):-
    findall(Tuple,
            (ask(triple(ID, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName',TtSName)),atom_string(TtSName,TtSNameString),
            string_lower(TtSNameString,TtSNameStringLower),
            Tuple = [ID,TtSNameStringLower]),
            TtsNames),
    findall(Tuple,
            (subclass_of(Nlong,'http://ias.cs.tum.edu/kb/knowrob.owl#EnduringThing-Localized'),split_string(Nlong, "#","",L), nth0(1,L,Name),string_lower(Name,NameLower), Tuple = [Nlong,NameLower])
            ,Names), append(TtsNames,Names,AllNamesWithDup),sort(AllNamesWithDup, AllNames).


matching_obj_classes(Name,[Class|Classes]):-
    all_obj_names_with_id(AllNames), member([Class,Name], AllNames), findall(SubClass, subclass_of(SubClass,Class), Classes).

matching_object_in_list(Name, Objects, Obj):-
    matching_obj_classes(Name,Classes), member(Obj,Objects), member(Class, Classes), ask(is_a(Obj,Class)).


%%%%%ROOMS%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

all_room_names(Names):-
    all_room_names_with_id(AllNames),findall(Name, member([_,Name],AllNames),Names).


all_room_names_with_id(AllNames):-
    findall(Tuple,
            (ask(triple(ID, 'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#TtSName',TtSName)),atom_string(TtSName,TtSNameString),
            string_lower(TtSNameString,TtSNameStringLower),
            Tuple = [ID,TtSNameStringLower]),
            TtsNames),
    findall(Tuple,
            (subclass_of(Nlong,'http://www.semanticweb.org/suturo/ontologies/2021/0/rooms#Room'),split_string(Nlong, "#","",L), nth0(1,L,Name),string_lower(Name,NameLower), Tuple = [Nlong,NameLower])
            ,Names), append(TtsNames,Names,AllNamesWithDup),sort(AllNamesWithDup, AllNames).


matching_room_class(Name,Class):-
    all_room_names_with_id(AllNames), member([Class,Name], AllNames).

matching_room(Name,Room):-    
    matching_room_class(Name,Class), all_rooms_of_type(Class,Rooms), member(Room, Rooms).
    
