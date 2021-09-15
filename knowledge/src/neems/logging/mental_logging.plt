:- module(general_logging, [
    init_logging/1,
    finish_logging/0,
    insert_knowledge_objects_logging/2,
    grasp_handling_logging/2,
    begin_action_logging/2,
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
    memorize('neem_plan'),
    writeln('===== ---> passed !').

xyz_logging(EpisodeID, ActionID) :-
    writeln('===== log: xyz'),
    %% ==== main logs
    tell(is_action(ActionXYZ)),
    tell(has_participant(ActionXYZ,'hsr')),
    tell(is_performed_by(ActionXYZ, 'hsr')),
    %% ==== type logs
    tell(has_type(SomeType, soma:'SomeTask')),
    %% ==== execute logs
    tell(executes_task(ActionXYZ, SomeType)),
    %% ==== further main logs
    ActionID = ActionXYZ,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').






insert_knowledge_objects_logging(EpisodeID, ActionID) :-
    writeln('===== log: insert knowledge objects'),
    %% ==== main logs
    tell(is_action(InsertObjectsAction)),
    tell(has_participant(InsertObjectsAction,'hsr')),
    tell(is_performed_by(InsertObjectsAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(InsertObjectsAction, MentalTask)),
    %% ==== further main logs
    ActionID = InsertObjectsAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').



grasp_handling_logging(EpisodeID, ActionID) :-
    writeln('===== log: grasp handling'),
    %% ==== main logs
    tell(is_action(GraspHandlingAction)),
    tell(has_participant(GraspHandlingAction,'hsr')),
    tell(is_performed_by(GraspHandlingAction, 'hsr')),
    %% ==== type logs
    tell(has_type(ManipulatinTask, soma:'Manipulating')),
    %% ==== execute logs
    tell(executes_task(GraspHandlingAction, ManipulatinTask)),
    %% ==== further main logs
    ActionID = GraspHandlingAction,
    tell(is_setting_for(EpisodeID, ActionID)),
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


