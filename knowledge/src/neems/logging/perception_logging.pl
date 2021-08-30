:- module(perception_logging, []).

%%% log_scan_table_sequence() is nondet.
%
% Log the necessary infos for the Scan Table Sequence Task.
%
log_scan_table_sequence :-
    writeln('===== log_scan_table_sequence'),
    tell(is_action(ScanTableSequence)),
    tell(has_participant(ScanTableSequence, 'hsr')),
    tell(is_performed_by(ScanTableSequence, 'hsr')),

    % get this running
    %get_time(Start),
    %get_time(End),
    %tell(occurs(SetupTask) during [Start, End]),

    % get from  navigation_loggin
    %tell(has_subevent(ScanTableSequence, MoveToTable)),
    %tell(has_subevent(ScanTableSequence, TakePoseAction)),

    tell(has_subevent(ScanTableSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanTableSequence, InsertKnowledgeObjects)),

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

:- writeln('Here in perception logging').