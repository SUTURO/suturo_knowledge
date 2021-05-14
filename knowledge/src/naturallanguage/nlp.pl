:- module(nlp,
    [
    get_names/1
    ]).

:- rdf_meta
    get_names(r).

get_names(AllNames):-
    findall(T,
            ask(triple(_, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#TtSName',T)),
            TtsNames),
    findall(Name,
            (subclass_of(Nlong,'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#PhysicalObject'),split_string(Nlong, "#","",L), nth0(1,L,Name))
            ,Names), append(TtsNames,Names,AllNamesWithDup),sort(AllNamesWithDup, AllNames).


matching_object(Name,Object):-
    
