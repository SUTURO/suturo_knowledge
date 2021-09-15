:- module(mental_logging, [
    insert_knowledge_objects_logging/2
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



% todo: loop_over_rooms_logging
% todo: next_object_logging
% todo: loop_over_surfaces_logging
% todo: set_surfaces_not_visited_logging
% todo: define_surfaces_logging