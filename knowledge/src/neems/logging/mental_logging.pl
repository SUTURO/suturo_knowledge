:- module(mental_logging, [
    insert_knowledge_objects_logging/2,
    loop_over_rooms_logging/2,
    next_object_logging/2,
    loop_over_surfaces_logging/2,
    set_surfaces_not_visited_logging/2,
    define_surfaces_logging/2
    ]).

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

%%% loop_over_rooms_logging() is det.
%
%
%
loop_over_rooms_logging(EpisodeID, ActionID) :-
    writeln('===== log: loop over rooms'),
    %% ==== main logs
    tell(is_action(LoopOverRoomsAction)),
    tell(has_participant(LoopOverRoomsAction,'hsr')),
    tell(is_performed_by(LoopOverRoomsAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(LoopOverRoomsAction, MentalTask)),
    %% ==== further main logs
    ActionID = LoopOverRoomsAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


%%% next_object_logging() is det.
%
%
%
next_object_logging(EpisodeID, ActionID) :-
    writeln('===== log: next object'),
    %% ==== main logs
    tell(is_action(NextObjectAction)),
    tell(has_participant(NextObjectAction,'hsr')),
    tell(is_performed_by(NextObjectAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(NextObjectAction, MentalTask)),
    %% ==== further main logs
    ActionID = NextObjectAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

%%% loop_over_surfaces_logging() is det.
%
%
%
loop_over_surfaces_logging(EpisodeID, ActionID) :-
    writeln('===== log: loop over surfaces'),
    %% ==== main logs
    tell(is_action(LoopSurfacesAction)),
    tell(has_participant(LoopSurfacesAction,'hsr')),
    tell(is_performed_by(LoopSurfacesAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(LoopSurfacesAction, MentalTask)),
    %% ==== further main logs
    ActionID = LoopSurfacesAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').

%%% set_surfaces_not_visited_logging() is det.
%
%
%
set_surfaces_not_visited_logging(EpisodeID, ActionID) :-
    writeln('===== log: set surfaces not visited'),
    %% ==== main logs
    tell(is_action(SurfaceNotVisitedAction)),
    tell(has_participant(SurfaceNotVisitedAction,'hsr')),
    tell(is_performed_by(SurfaceNotVisitedAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(SurfaceNotVisitedAction, MentalTask)),
    %% ==== further main logs
    ActionID = SurfaceNotVisitedAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').


%%% define_surfaces_logging() is det.
%
%
%
define_surfaces_logging(EpisodeID, ActionID) :-
    writeln('===== log: define surfaces'),
    %% ==== main logs
    tell(is_action(DefineSurfacesAction)),
    tell(has_participant(DefineSurfacesAction,'hsr')),
    tell(is_performed_by(DefineSurfacesAction, 'hsr')),
    %% ==== type logs
    tell(has_type(MentalTask, soma:'MentalTask')),
    %% ==== execute logs
    tell(executes_task(DefineSurfacesAction, MentalTask)),
    %% ==== further main logs
    ActionID = DefineSurfacesAction,
    tell(is_setting_for(EpisodeID, ActionID)),
    begin_action_logging(ActionID),
    writeln('===== ---> passed !').