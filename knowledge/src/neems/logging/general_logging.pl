:- module(general_logging, [
    init_logging/1,
    finish_logging/0,
    scan_floor_logging/0,
    scan_floor_logging/1,
    search_surfaces_logging/0,
    not_visited_logging/0
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
    get_time(EndTime),
    atom_concat(NeemPath,'/',X1),
    atom_concat(X1,EndTime,X2),
    memorize(X2),
    memorize('neem_plan_1'),
    writeln('===== ---> passed !').

%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
% todo: add EpisodeID
scan_floor_logging(EpisodeID) :-
    writeln('===== log: scan floor'),
    writeln(['===== print: EpisodeID is: ', EpisodeID]),
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

% todo:    move_hsr_logging(ID)
% todo:    take_pose_action_logging(ID)
% todo:    insert_knowledge_objects_logging(ID)
% todo:    call_take_pose_action_logging(ID)
% todo:    grasp_handling_logging(ID)

