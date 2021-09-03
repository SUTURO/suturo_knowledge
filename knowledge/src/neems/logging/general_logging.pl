:- module(general_logging, [
    init_logging/0,
    finish_logging/0,
    scan_floor_logging/0,
    search_surfaces_logging/0,
    not_visited_logging/0
    ]).

%% ========================= all logs for neem-plan-1


%%% neem_init() is det.
%
% Initialize the neem.
%
init_logging :-
    writeln('===== neem_init'),
    tf_logger_enable,
    tell([
        is_episode(NeemPlan1Episode),
        is_setting_for(NeemPlan1Episode, 'hsr')
    ]),
    writeln('===== ---> passed !').

%%% neem_init() is det.
%
% Initialize the neem.
%
init_logging(NeemStart) :-
    writeln('===== neem_init'),
    NeemStart = 31,
    tf_logger_enable,
    tell([
        is_episode(NeemPlan1Episode),
        is_setting_for(NeemPlan1Episode, 'hsr')
    ]),
    writeln('===== ---> passed !').

%%% finish_logging() is det.
%
% Conclude the neem.
%
finish_logging :-
    writeln('===== neem_terminate'),
    get_time(EndTime),
    atom_concat(NeemPath,'/',X1),
    atom_concat(X1,EndTime,X2),
    memorize(X2),
    memorize('neem_plan_1'),
    writeln('===== ---> passed !').

%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
%
scan_floor_logging :-
    writeln('===== log: scan floor'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),

    % get this running
    %get_time(Start),
    %get_time(End),
    %tell(occurs(SetupTask) during [Start, End]),

    % get from  navigation_loggin
    %tell(has_subevent(ScanTableSequence, MoveToTable)),
    %tell(has_subevent(ScanTableSequence, TakePoseAction)),

    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),

    % get from navigation_logging
    %tell(has_type(NavTask, soma:'Navigating')),
    %tell(has_type(PoseTask, soma:'AssumingPose')),

    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),

    % get from navigation_logging
    %tell(executes_task(MoveToTable, NavTask)),
    %tell(executes_task(TakePoseAction, PoseTask)),

    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    writeln('===== ---> passed !').

%% ========================= all logs for neem-plan-2


%%%
%
%
%
search_surfaces_logging :-
    writeln('===== log: search surfaces'),
    tell(is_action(SearchSurfaces)),
    tell(has_participant(SearchSurfaces, 'hsr')),
    tell(is_performed_by(SearchSurfaces, 'hsr')),
    tell(has_type(SearchSurfaces, soma:'Reasoning')),
    writeln('===== ---> passed !').

%%%
%
%
%
not_visited_logging :-
    writeln('===== log: setting surfaces as not visited'),
    tell(is_action(SetSurfaces)),
    tell(has_participant(SetSurfaces, 'hsr')),
    tell(is_performed_by(SetSurfaces, 'hsr')),
    tell(has_type(SetSurfaces, soma:'Reasoning')),
    writeln('===== ---> passed !').

%% ========================= other



%%% log_setup() is nondet.
%
% Log the necessary infos for some Setup Task.
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