:- module(navigation_logging, [
    move_hsr_logging/2
    ]).


%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
move_hsr_logging(EpisodeID, ActionID) :-
    writeln('===== log: move hsr'),
    %% ==== main logs
    tell(is_action(MoveHsrAction)),
    tell(has_participant(MoveHsrAction,'hsr')),
    tell(is_performed_by(MoveHsrAction, 'hsr')),
    %% ==== type logs
    tell(has_type(NavigationTask, soma:'NavigationTask')),
    %% ==== execute logs
    tell(executes_task(MoveHsrAction, NavigationTask)),
    %% ==== further main logs
    ActionID = MoveHsrAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

% todo: move_to_surface_logging
% todo: deliver_object_logging