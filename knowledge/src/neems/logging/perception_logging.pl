:- module(perception_logging, [
    scan_floor_logging/2,
    perceive_surface_logging/2
    ]).

%%% scan_floor_logging() is det.
%
% Log the actions to scan the floor.
%
scan_floor_logging(EpisodeID, ActionID) :-
    writeln('===== log: scan floor'),
    %% ==== main logs
    tell(is_action(ScanFloorAction)),
    tell(has_participant(ScanFloorAction,'hsr')),
    tell(is_performed_by(ScanFloorAction, 'hsr')),
    %% ==== type logs
    tell(has_type(PhysicalTask, soma:'PhysicalTask')),
    %% ==== execute logs
    tell(executes_task(ScanFloorAction, PhysicalTask)),
    %% ==== further main logs
    ActionID = ScanFloorAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


%%% perceive_surface_logging() is det.
%
%
%
perceive_surface_logging(EpisodeID, ActionID) :-
    writeln('===== log: perceive surface'),
    %% ==== main logs
    tell(is_action(PerceiveSurfaceAction)),
    tell(has_participant(PerceiveSurfaceAction,'hsr')),
    tell(is_performed_by(PerceiveSurfaceAction, 'hsr')),
    %% ==== type logs
    tell(has_type(PhysicalTask, soma:'PhysicalTask')),
    %% ==== execute logs
    tell(executes_task(PerceiveSurfaceAction, PhysicalTask)),
    %% ==== further main logs
    ActionID = PerceiveSurfaceAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').