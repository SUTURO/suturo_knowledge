%% The Util module contains predicates that (currently) don't fit into other modules.
:- module(util,
	  [
        from_current_scope(-),
        split_iri(+, -, -),
        is_bnode(+),
        last_element(+, -),
        second_last_element(+, -),
        ros_info(+,+),
        ros_warn(+,+),
        ros_error(+,+),
        ros_debug(+,+),
        default_value(?,+)
	  ]).

:- use_module(library('semweb/rdf_db')).

:- use_module(library('lang/terms/triple'),
	      [
		  triple/3
	      ]).

%% from_current_scope(-Scope) is det.
%
% The scope of facts that are true from now until infinity.
%
% @param Scope A scope dictionary.
%
from_current_scope(dict{
		       time: dict{
				 since: =(double(Now)),
				 until: =(double('Infinity'))
	}
}) :- get_time(Now).

%% default_value(?Term, +DefaultValue) is det.
%
% if Term is a var, unify it with DefaultValue otherwise leave it as it is.
%
% @param Term The term to unify
% @param DefaultValue The value to unify with
%
default_value(Term, DefaultValue) :-
    (var(Term)
    -> Term = DefaultValue
    ;  true).


% split_iri(+IRI, -Prefix, -ClassIdentifier)
% 
% Splits an IRI of the form <OntologyURI>#<ClassIdentifier> into the prefix part and the class identifier.
% 
% @param IRI The IRI to split
% @param OntologyURI The prefix part of the IRI, usually the namespace/ontology uri
% @param ClassIdentifier The class identifier part of the IRI
% 
split_iri(IRI, OntologyURI, ClassIdentifier) :-
    sub_atom(IRI, Before, _, After, '#'),
    !,
    sub_atom(IRI, 0, Before, _, OntologyURI),
    sub_atom(IRI, _, After, 0, ClassIdentifier).


%% is_bnode(+IRI) is det.
%
% True if the given IRI is a blank node.
%
% @param IRI The IRI to check
%
is_bnode(IRI) :-
    split_iri(IRI, _, ClassIdentifier),
    rdf_is_bnode(ClassIdentifier).

%% last_element(+List, -LastElement) is det.
%
% Returns the last element of a list
%
% @param List List of elements
% @param LastElement Last element of the list
%
last_element([X], X).
last_element([_|T], X) :- 
    last_element(T, X).

%% second_last_element(+List, -SecondLastElement) is det.
%
% Returns the second last element of a list
%
% @param List List of elements
% @param SecondLastElement Second last element of the list
%
second_last_element([X,_], X).
second_last_element([_|T], X) :- 
    second_last_element(T, X).

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Debugging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ros_debug(+Format, +Arguments)
%
% Logs a debug message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_debug(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_debug(MSG).

%% ros_info(+Format, +Arguments)
%
% Logs an info message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_info(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_info(MSG).

%% ros_warn(+Format, +Arguments)
%
% Logs a warning message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_warn(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_warn(MSG).

%% ros_error(+Format, +Arguments)
%
% Logs an error message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_error(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_error(MSG).