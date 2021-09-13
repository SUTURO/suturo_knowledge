:- module(general_logging, [
    init_logging/1,
    finish_logging/0,
    scan_floor_logging/2,
    move_hsr_logging/1,
    take_pose_action_logging/1,
    insert_knowledge_objects_logging/1,
    call_take_pose_action_logging/1,
    grasp_handling_logging/1,
    begin_action_logging/1,
    end_action_logging/1,
    testing_logs/0
    ]).

%%% neem_init() is det.
%
% Initialize the neem.
%
init_logging(EpisodeID) :-
    writeln('===== init_logging'),
    tf_logger_enable,
    tell([
        is_episode(Episode),
        is_setting_for(Episode, 'hsr')
    ]),
    EpisodeID = Episode,
    writeln('===== ---> passed !').


%%% finish_logging() is det.
%
% Conclude the neem.
%
finish_logging :-
    writeln('===== neem_terminate'),
%    get_time(EndTime),
%    atom_concat(NeemPath,'/',X1),
%    atom_concat(X1,EndTime,X2),
%    memorize(X2),
    memorize('neem_plan_1'),
    writeln('===== ---> passed !').

%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
scan_floor_logging(EpisodeID, ActionID) :-
    writeln('===== log: scan floor'),
    writeln(['===== print: EpisodeID is: ', EpisodeID]),
    tell(is_action(ScanFloorSequence)),
    writeln('here after is_action'),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    writeln('here after has_subevent'),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    writeln('here after executes_task'),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    writeln('here after is_setting_for'),
    ActionID = ScanFloorSequence,
    writeln('here after ActionID = (...)'),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
move_hsr_logging(EpisodeID, ActionID) :-
    writeln('===== log: move hsr'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    ActionID = ScanFloorSequence,
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

take_pose_action_logging(EpisodeID, ActionID) :-
    writeln('===== log: take pose action'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    ActionID = ScanFloorSequence,
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


insert_knowledge_objects_logging(EpisodeID, ActionID) :-
    writeln('===== log: insert knowledge objects'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    ActionID = ScanFloorSequence,
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').




call_take_pose_action_logging(EpisodeID, ActionID) :-
    writeln('===== log: call take pose action'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    ActionID = ScanFloorSequence,
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

grasp_handling_logging(EpisodeID, ActionID) :-
    writeln('===== log: grasp handling'),
    tell(is_action(ScanFloorSequence)),
    tell(has_participant(ScanFloorSequence, 'hsr')),
    tell(is_performed_by(ScanFloorSequence, 'hsr')),
    tell(has_subevent(ScanFloorSequence, GetConfidenceObjects)),
    tell(has_subevent(ScanFloorSequence, InsertKnowledgeObjects)),
    tell(has_type(PerceiveTask, soma:'Perceiving')),
    tell(has_type(ReasoningTask, soma:'Reasoning')),
    tell(executes_task(GetConfidenceObjects, PerceiveTask)),
    tell(executes_task(InsertKnowledgeObjects, ReasoningTask)),
    tell(is_setting_for(EpisodeID, ScanFloorSequence)),
    ActionID = ScanFloorSequence,
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


% ===================================

% Largely taken from CCL/neem-interface.pl
begin_action_logging(X) :-
    writeln('Here in begin action logging'),
    get_time(Begin),
    tell(occurs(X) since Begin),
    !.

end_action_logging(ActionID) :-
 writeln('here in end_action_logging'),
 get_time(CurrentTime),
 ask(triple(ActionID,dul:'hasTimeInterval',TimeInterval)),
 tripledb_forget(TimeInterval, soma:'hasIntervalEnd', _),
 tell(holds(TimeInterval, soma:'hasIntervalEnd', CurrentTime)),
 writeln(['==== ending Action: ', ActionID]),
 !.



% 'http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_BHSEMVGN'

% ask(triple('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_BHSEMVGN', X,Y))

% get_time(Ending),
% ask(triple('http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#Action_BHSEMVGN', dul:'hasTimeInterval',TimeInterval)),
%

testing_logs :-
    init_logging(EpisodeID),
    %% ===
    scan_floor_logging(EpisodeID, A),
    end_action_logging(A),
    %% ===
    move_hsr_logging(EpisodeID, B),
    end_action_logging(B),
    %% ===
    call_take_pose_action_logging(EpisodeID, C),
    end_action_logging(C),
    %% ===
    insert_knowledge_objects_logging(EpisodeID, D),
    end_action_logging(D),
    %% ===
    call_take_pose_action_logging(EpisodeID, E),
    end_action_logging(E),
    %% ===
    move_hsr_logging(EpisodeID, F),
    end_action_logging(F),
    %% ===
    grasp_handling_logging(EpisodeID, G),
    end_action_logging(G),
    %% ===
    finish_logging.


