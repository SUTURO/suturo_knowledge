:- module(reset, [
			  reset_user_data/0
		  ]).

%% reset_user_data is det.
%
% drop the user graph (containing objects and data assigned at runtime),
% drop all tf data,
% and reinitialize the furnitures from the semantic map.
reset_user_data :-
    drop_graph(user),
    tf_mng_drop,
    tf_mem_clear,
    % if the tf logger is not reinitialized, the static transforms will be missing.
    tf_logger_enable,
    init_rooms,
    init_furnitures.
