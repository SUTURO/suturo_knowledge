:- module(nlp,
    [
    all_names/1,
    all_names_with_id/1,
    matching_classes/2,
    matching_object/2
    ]).

:- rdf_meta
    all_names(r),
    all_names_with_id(r),
    matching_classes(r,r).

all_names(Names):-
    all_names_with_id(AllNames),findall(Name, member([_,Name],AllNames),Names).

all_names_with_id(AllNames):-
    findall(Tuple,
            (ask(triple(ID, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName',TtSName)),atom_string(TtSName,TtSNameString),
            string_lower(TtSNameString,TtSNameStringLower),
            Tuple = [ID,TtSNameStringLower]),
            TtsNames),
    findall(Tuple,
            (subclass_of(Nlong,'http://ias.cs.tum.edu/kb/knowrob.owl#EnduringThing-Localized'),split_string(Nlong, "#","",L), nth0(1,L,Name),string_lower(Name,NameLower), Tuple = [Nlong,NameLower])
            ,Names), append(TtsNames,Names,AllNamesWithDup),sort(AllNamesWithDup, AllNames).


matching_classes(Name,Classes):-
    all_names_with_id(AllNames), member([Class,Name], AllNames), findall(SubClass, subclass_of(SubClass,Class), Classes).

matching_object(Name, Obj):-
    matching_classes(Name,Classes), hsr_existing_objects(Objects), member(Obj,Objects), member(Class, Classes), ask(is_a(Obj,Class)).
