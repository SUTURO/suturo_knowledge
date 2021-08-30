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
:- writeln('Here in older neems').

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
    writeln('===== neem_init'),
    tf_logger_enable,
    tell([
        is_episode(GroceriesEpisode),
        is_setting_for(GroceriesEpisode, 'hsr')
    ]),
    writeln('===== ---> passed !').

%%% log_setup() is nondet.
%
% Log the necessary infos for the Setup Task.
%
log_setup :-
    writeln('===== log_setup'),
    tell(is_action(SetupTask)),
    tell(has_participant(SetupTask, 'hsr')),
    tell(is_performed_by(SetupTask, 'hsr')),
    % get_time(Start),
    % get_time(End),
    % tell(occurs(SetupTask) during [Start, End]),
    tell(has_subevent(SetupTask, SetTableSource)),
    tell(has_subevent(SetupTask, SetTargetSurface)),
    tell(has_subevent(SetupTask, TakePoseAction)),
    tell(has_type(Task1, soma:'GetTaskParameter')),
    tell(has_type(Task2, soma:'AssumingPose')),
    tell(executes_task(SetTableSource, Task1)),
    tell(executes_task(SetTargetSurface, Task1)),
    tell(executes_task(TakePoseAction, Task2)),
    writeln('===== ---> passed !').


%%% log_scan_table_sequence() is nondet.
%
% Log the necessary infos for the Scan Table Sequence Task.
%
log_scan_table_sequence :-
    writeln('===== log_scan_table_sequence'),
    tell(is_action(ScanTableSequence)),
    tell(has_participant(ScanTableSequence, 'hsr')),
    tell(is_performed_by(ScanTableSequence, 'hsr')),
    % get_time(Start),
    % get_time(End),
    % tell(occurs(SetupTask) during [Start, End]),
    tell(has_subevent(ScanTableSequence, MoveToTable)),
    tell(has_subevent(ScanTableSequence, TakePoseAction)),
    tell(has_subevent(ScanTableSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanTableSequence, InsertKnowledgeObjects)),
    tell(has_type(NavTask, soma:'Navigating')),
    tell(has_type(PoseTask, soma:'AssumingPose')),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(MoveToTable, NavTask)),
    tell(executes_task(TakePoseAction, PoseTask)),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    writeln('===== ---> passed !').

%%% log_grasp_sequence() is nondet.
%
% Log the necessary infos for the Grasping Sequence Task.
%
log_grasp_sequence :-
    writeln('===== log_grasp_sequence'),
    tell(is_action(GraspSequence)),
    tell(has_participant(GraspSequence, 'hsr')),
    tell(is_performed_by(GraspSequence, 'hsr')),
    % get_time(Start),
    % get_time(End),
    % tell(occurs(SetupTask) during [Start, End]),
    tell(has_subevent(GraspSequence, PrologNextObject)),
    tell(has_subevent(GraspSequence, GraspObject)),
    tell(has_subevent(GraspSequence, MoveToTable)),
    tell(has_subevent(GraspSequence, PrologForgetTableObjects)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(has_type(GrapTask, soma:'Grasping')),
    tell(has_type(NavTask, soma:'Navigating')),
    tell(executes_task(PrologNextObject, ReasoningTask)),
    tell(executes_task(GraspObject, GrapTask)),
    tell(executes_task(MoveToTable, NavTask)),
    tell(executes_task(PrologForgetTableObjects, ReasoningTask)),
    writeln('===== ---> passed !').

%%% log_scan_shelf_floors_sequence() is nondet.
%
% Log the necessary infos for the Scan Shelf floors Sequence Task.
%
log_scan_shelf_floors_sequence :-
    writeln('===== log_scan_shelf_floors_sequence'),
    tell(is_action(ScanShelfFloorsSequence)),
    tell(has_participant(ScanShelfFloorsSequence, 'hsr')),
    tell(is_performed_by(ScanShelfFloorsSequence, 'hsr')),
    % get_time(Start),
    % get_time(End),
    % tell(occurs(SetupTask) during [Start, End]),
    tell(has_subevent(ScanShelfFloorsSequence, PerceiveShelf)),
    tell(has_subevent(ScanShelfFloorsSequence, TakePoseAction)),
    tell(has_subevent(ScanShelfFloorsSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanShelfFloorsSequence, InsertKnowledgeObjects)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(PoseTask, soma:'AssumingPose')),
    tell(executes_task(PerceiveShelf, PerceiveTask)),
    tell(executes_task(TakePoseAction, PoseTask)),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    writeln('===== ---> passed !').

%%% log_place_sequence() is nondet.
%
% Log the necessary infos for the Place Sequence Task.
%
log_place_sequence :-
    writeln('===== log_place_sequence'),
    tell(is_action(PlaceSequence)),
    tell(has_participant(PlaceSequence, 'hsr')),
    tell(is_performed_by(PlaceSequence, 'hsr')),
    % get_time(Start),
    % get_time(End),
    % tell(occurs(SetupTask) during [Start, End]),
    tell(has_subevent(PlaceSequence, PrologShelfPosition)),
    tell(has_subevent(PlaceSequence, PrologObjectGoal)),
    tell(has_subevent(PlaceSequence, TakePoseAction)),
    tell(has_subevent(PlaceSequence, PrologTablePose)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(has_type(PoseTask, soma:'AssumingPose')),
    tell(executes_task(PrologShelfPosition, ReasoningTask)),
    tell(executes_task(PrologObjectGoal, ReasoningTask)),
    tell(executes_task(TakePoseAction, PoseTask)),
    tell(executes_task(PrologTablePose, ReasoningTask)),
    writeln('===== ---> passed !').

%%% neem_term() is det.
%
% End the Neem and save it into destinated folder.
%
neem_terminate :-
    writeln('===== neem_terminate'),
    memorize('storing_groceries_neem'),
    writeln('===== ---> passed !').

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

