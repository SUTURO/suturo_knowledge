:- module(manipulation_logging, [
    call_take_pose_action_logging/2,
    grasp_handling_logging/2
    ]).

call_take_pose_action_logging(EpisodeID, ActionID) :-
    writeln('===== log: call take pose action'),
    %% ==== main logs
    tell(is_action(CallTakePoseAction)),
    tell(has_participant(CallTakePoseAction,'hsr')),
    tell(is_performed_by(CallTakePoseAction, 'hsr')),
    %% ==== type logs
    tell(has_type(AssumingPoseTask, soma:'AssumingPose')),
    %% ==== execute logs
    tell(executes_task(CallTakePoseAction, AssumingPoseTask)),
    %% ==== further main logs
    ActionID = CallTakePoseAction,
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

% todo: handle_objects_logging