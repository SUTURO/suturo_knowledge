:- use_module(library('db/tripledb_tests')).

:- use_module(library('knowrob')).
:- use_module(library('semweb/rdf_db')).
:- use_module(library('semweb/rdfs')).

:- begin_tripledb_tests(
      	'locations',
       	'package://knowledge/owl/testing.owl',
       	[ namespace('http://www.semanticweb.org/suturo/ontologies/2021/0/testing#')]
 ).


 test(get_location) :-
    true.

 :- end_tripledb_tests('locations').