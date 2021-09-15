:- module(navigation_logging, [
    move_hsr_logging/2,
    move_to_surface_logging/2,
    deliver_object_logging/2
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

%%% move_to_surface_logging() is det.
%
%
move_to_surface_logging(EpisodeID, ActionID) :-
    writeln('===== log: move to surface'),
    %% ==== main logs
    tell(is_action(MoveToSurfaceAction)),
    tell(has_participant(MoveToSurfaceAction,'hsr')),
    tell(is_performed_by(MoveToSurfaceAction, 'hsr')),
    %% ==== type logs
    tell(has_type(NavigationTask, soma:'NavigationTask')),
    %% ==== execute logs
    tell(executes_task(MoveToSurfaceAction, NavigationTask)),
    %% ==== further main logs
    ActionID = MoveToSurfaceAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


%%% deliver_object_logging() is det.
%
%
deliver_object_logging(EpisodeID, ActionID) :-
    writeln('===== log: deliver object'),
    %% ==== main logs
    tell(is_action(DeliverObjectAction)),
    tell(has_participant(DeliverObjectAction,'hsr')),
    tell(is_performed_by(DeliverObjectAction, 'hsr')),
    %% ==== type logs
    tell(has_type(NavigationTask, soma:'NavigationTask')),
    %% ==== execute logs
    tell(executes_task(DeliverObjectAction, NavigationTask)),
    %% ==== further main logs
    ActionID = DeliverObjectAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').