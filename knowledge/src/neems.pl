:- module(neems,[
    neem_init/0,
    log_setup/0,
    log_scan_table_sequence/0,
    log_grasp_sequence/0,
    log_scan_shelf_floors_sequence/0,
    log_place_sequence/0,
    neem_terminate/0,
    episode_overview/0,
    event_overview/0,
    action_overview/0,
    event_overview/0,
    action_overview/0,
    object_overview/0,
    get_first_event/0,
    neem_storing_groceries/0,
    neem_query/0
    ]).

%%% ====================================== load ontologies
:- rdf_db:rdf_register_ns(hsr_objects, 'http://www.semanticweb.org/suturo/ontologies/2020/3/objects#', [keep(true)]).
:- rdf_db:rdf_register_ns(robocup, 'http://www.semanticweb.org/suturo/ontologies/2020/2/Robocup#', [keep(true)]).
:- rdf_db:rdf_register_ns(dul, 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(soma, 'https://ease-crc.github.io/soma/owl/current/SOMA.owl#', [keep(true)]).
:- rdf_db:rdf_register_ns(knowrob, 'http://www.knowrob.org/kb/knowrob.owl#', [keep(true)]).


%%% ====================================== helper functions
neem_storing_groceries :-
    neem_init,
    log_setup,
    log_scan_table_sequence,
    log_grasp_sequence,
    log_scan_shelf_floors_sequence,
    log_place_sequence,
    neem_terminate.

neem_query :-
    remember('storing_groceries_neem'),
    episode_overview,
    event_overview,
    action_overview,
    object_overview.

%%% ====================================== neem log predicates (for now only Storing Groceries)
%%% neem_init() is det.
%
% Initialize the neem.
%
neem_init :-
    tf_logger_enable.
    tell([
        Episode = 'Storing Groceries Main Episode',
        Agent = 'hsr',
        is_episode(Episode),
        is_setting_for(Episode, Agent)
    ]).

%%% log_setup() is nondet.
%
% Log the necessary infos for the Setup Task.
%
log_setup :-
    writeln('===== log_setup'),
    writeln('tell is action'),
    tell(is_action('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action')),
    writeln('===== ---> passed !').

%%% log_scan_table_sequence() is nondet.
%
% Log the necessary infos for the Scan Table Sequence Task.
%
log_scan_table_sequence :-
    X = 31.

%%% log_grasp_sequence() is nondet.
%
% Log the necessary infos for the Grasping Sequence Task.
%
log_grasp_sequence :-
    X = 31.

%%% log_scan_shelf_floors_sequence() is nondet.
%
% Log the necessary infos for the Scan Shelf floors Sequence Task.
%
log_scan_shelf_floors_sequence :-
    X = 31.

%%% log_place_sequence() is nondet.
%
% Log the necessary infos for the Place Sequence Task.
%
log_place_sequence :-
    X = 31.

%%% neem_term() is det.
%
% End the Neem and save it into destinated folder.
%
neem_terminate :-
    memorize('storing_groceries_neem').

%%% ====================================== neem query predicates

%%% episode_overview() is nondet.
%
% Give an expressive representation of the existing Episodes.
%
episode_overview :-
    is_episode(Event).

%%% event_overview() is nondet.
%
% Give an expressive representation of the existing Events.
%
event_overview :-
    is_event(Event).

%%% action_overview() is nondet.
%
% Give an expressive representation of the existing Actions.
%
action_overview :-
    X = 31.

%%% object_overview() is nondet.
%
% Give an expressive representation of the existing Objects.
%
object_overview :-
    X = 31.


%%% get_first_event() is nondet.
%
% Return the first Event.
%
get_first_event :-
    X = 31.

