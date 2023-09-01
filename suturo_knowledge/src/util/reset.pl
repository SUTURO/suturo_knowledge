:- module(reset, [
			  reset_user_data/0
		  ]).

:- use_module(library(lang/query), []). %% default_graph/1

%% reset_user_data is det.
%
% drop the user graph (containing objects and data assigned at runtime),
% drop all tf data,
% and reinitialize the furnitures from the semantic map.
reset_user_data :-
    lang_query:default_graph(Graph),
    drop_graph(Graph),
    tf_mng_drop,
    tf_mem_clear,
    % if the tf logger is not reinitialized, the static transforms will be missing.
    tf_logger_enable,
    init_rooms,
    init_predefined_names_robocup_2023,
    init_furnitures.
