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

%%% start_action() is nondet.
%
% Create the minal triples that represent an action.
%
%
start_action(Action, RobotName):-
    is_action(Action),
    tell(has_participant(Action, RobotName)),
    tell(is_performed_by(Action, RobotName)).

%%% set_time_index() is nondet.
%
% Set the current action in relation to the time index.
%
%
set_time_index(Start, End, Action):-
     notify_synchronize(event(Action)),
     tell(occurs(Action) during [Start, End]).

%%% add_subevents() is nondet.
%
% Add Subaction to the given Task.
%
%
add_subevents(Action, Subaction):-
     tell(has_subevent(Action, Subaction)).

%%% make_task_parameter() is nondet.
%
% Create an instance of SOMA: GetTaskParameter for later usage.
%
%
make_task_parameter(Task) :-
    tell(has_type(Task, soma:'GetTaskParameter')).

%%% make_task_asuming_pose() is nondet.
%
% Create an instance of SOMA: AssumingPose for later usage.
%
%
make_task_asuming_pose(Task) :-
    tell(has_type(Task, soma:'AssumingPose')).

%%% make_task_navigation() is nondet.
%
% Create an instance of SOMA: Navigating for later usage.
%
%
make_task_navigation(Task) :-
    tell(has_type(Task, soma:'Navigating')).

%%% make_task_perception() is nondet.
%
% Create an instance of SOMA: Perceiving for later usage.
%
%
make_task_perception(Task) :-
    tell(has_type(Task, soma:'Perceiving')).

%%% task_GetTaskParameter() is nondet.
%
% Make the passed Action more representative by adding a suitable Task.
%
%
set_action_task(Action,Task) :-
    tell(executes_task(Action, Task)).

%%% ====================================== neem initialization & termination
%%% neem_init() is det.
%
% Initialize the neem.
%
neem_init :-
    rosinfo('===== neem_init'),
    tf_logger_enable,
    tell([
        is_episode(GroceriesEpisode),
        is_setting_for(GroceriesEpisode, 'hsr')
    ]),
    rosinfo('===== ---> passed !').

%%% neem_term() is det.
%
% End the Neem and save it into destinated folder.
%
neem_terminate :-
    rosinfo('===== neem_terminate'),
    memorize('storing_groceries_neem'),
    rosinfo('===== ---> passed !').

%%% ====================================== neem log predicates (for now only Storing Groceries)
%%% log_setup() is nondet.
%
% Log the necessary infos for the Setup Task.
%
%
log_setup(RobotName, Start, End) :-
    rosinfo('===== log_setup'),
    start_action(SetupTask,RobotName),
    set_time_index(Start, End, SetupTask),
    add_subevent(SetupTask, SetTableSource),
    add_subevent(SetupTask, SetTargetSurface),
    add_subevent(SetupTask, TakePoseAction),
    make_task_parameter(GetTaskParameter),
    make_task_asuming_pose(AssumingPose),
    set_action_task(SetTableSource, GetTaskParameter),
    set_action_task(SetTargetSurface, GetTaskParameter),
    set_action_task(TakePoseAction, AssumingPose),
    rosinfo('===== ---> passed !').


%%% log_scan_table_sequence() is nondet.
%
% Log the necessary infos for the Scan Table Sequence Task.
%
log_scan_table_sequence(RobotName,Start, End) :-
    rosinfo('===== log_scan_table_sequence'),
    start_action(ScanTableSequence, RobotName),
    set_time_index(Start, End, ScanTableSequence),
    add_subevent(ScanTableSequence, MoveToTable),
    add_subevent(ScanTableSequence, TakePoseAction),
    add_subevent(ScanTableSequence, GetConfidenceObjects),
    add_subevent(ScanTableSequence, InsertKnowledgeObjects),
    make_task_navigation(NavTask),
    make_task_asuming_pose(PoseTask),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    set_action_task(MoveToTable, NavTask)),
    set_action_task(TakePoseAction, PoseTask)),
    set_action_task(GetConfidenceObjects, PerceiveTask)),
    set_action_task(InsertKnowledgeObjects, ReasoningTask)),
    rosinfo('===== ---> passed !').

%%% log_grasp_sequence() is nondet.
%
% Log the necessary infos for the Grasping Sequence Task.
%
log_grasp_sequence :-
    rosinfo('===== log_grasp_sequence'),
    start_action(GraspSequence, RobotName)),
    tell(has_participant(GraspSequence, 'hsr')),
    tell(is_performed_by(GraspSequence, 'hsr')),
    add_subevent(GraspSequence, PrologNextObject)),
    add_subevent(GraspSequence, GraspObject)),
    add_subevent(GraspSequence, MoveToTable)),
    add_subevent(GraspSequence, PrologForgetTableObjects)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(has_type(GrapTask, soma:'Grasping')),
    make_task_navigation(NavTask),
    set_action_task(PrologNextObject, ReasoningTask)),
    set_action_task(GraspObject, GrapTask)),
    set_action_task(MoveToTable, NavTask)),
    set_action_task(PrologForgetTableObjects, ReasoningTask)),
    rosinfo('===== ---> passed !').

%%% log_scan_shelf_floors_sequence() is nondet.
%
% Log the necessary infos for the Scan Shelf floors Sequence Task.
%
log_scan_shelf_floors_sequence :-
    rosinfo('===== log_scan_shelf_floors_sequence'),
    start_action(ScanShelfFloorsSequence, RobotName)),
    tell(has_participant(ScanShelfFloorsSequence, 'hsr')),
    tell(is_performed_by(ScanShelfFloorsSequence, 'hsr')),
    add_subevent(ScanShelfFloorsSequence, PerceiveShelf)),
    add_subevent(ScanShelfFloorsSequence, TakePoseAction)),
    add_subevent(ScanShelfFloorsSequence, GetConfidenceObjects)),
    add_subevent(ScanShelfFloorsSequence, InsertKnowledgeObjects)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    make_task_asuming_pose(PoseTask),
    set_action_task(PerceiveShelf, PerceiveTask)),
    set_action_task(TakePoseAction, PoseTask)),
    set_action_task(GetConfidenceObjects, PerceiveTask)),
    set_action_task(InsertKnowledgeObjects, ReasoningTask)),
    rosinfo('===== ---> passed !').

%%% log_place_sequence() is nondet.
%
% Log the necessary infos for the Place Sequence Task.
%
log_place_sequence :-
    rosinfo('===== log_place_sequence'),
    start_action(PlaceSequence, RobotName)),
    tell(has_participant(PlaceSequence, 'hsr')),
    tell(is_performed_by(PlaceSequence, 'hsr')),
    add_subevent(PlaceSequence, PrologShelfPosition)),
    add_subevent(PlaceSequence, PrologObjectGoal)),
    add_subevent(PlaceSequence, TakePoseAction)),
    add_subevent(PlaceSequence, PrologTablePose)),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    make_task_asuming_pose(PoseTask),
    set_action_task(PrologShelfPosition, ReasoningTask)),
    set_action_task(PrologObjectGoal, ReasoningTask)),
    set_action_task(TakePoseAction, PoseTask)),
    set_action_task(PrologTablePose, ReasoningTask)),
    rosinfo('===== ---> passed !').


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

