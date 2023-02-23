%% The object giskard info module contains predicates are used to syncronise the world state of knowledge and giskard
:- module(object_giskard_info,
	  [
              giskard_updates/3,
	      giskard_test/0
	  ]).

%% giskard_updates(+LastUpdateTime, -CurrentUpdateTime, -ObjectData) is det.
%
% giskard_updates checks the db for objects that have been created or deleted and returns the information in the ObjectData parameter.
%
% this predicate is not implemented yet and just returns an empty list
%
% @param LastUpdateTime The last time the clients data got updated, or 0.0 if this is the first update
% @param CurrentUpdateTime The time at which the current updated gets processed.
%                          This value should be the LastUpdateTime in the next call
% @param ObjectData The list of updated objects. The format each object is described with is not specified yet.
giskard_updates(LastUpdateTime, CurrentUpdateTime, ObjectData) :-
    get_time(CurrentUpdateTime),
    % This time scope means: Data that started after LastUpdateTime and ends after CurrentUpdateTime
    time_scope(>=(LastUpdateTime), >(CurrentUpdateTime), Scope),
    findall(Object,
	    (
		kb_call(triple(Object, suturo:isManagedBy, suturo_knowledge), Scope, _FScope)
	    ),
	    ObjectList),
    maplist(giskard_object_shape, ObjectList, ObjectData),
    !.

%% giskard_object_shape(+Object, -Shape) is semidet.
%
% giskard_object_shape looks up the shape of the object and returns the data that should be send to giskard
% For information about the format of the result data, look at the documentation of giskard_updates/3.
%
% @param Object An Object IRI
% @param Shape The shape data used by giskard
giskard_object_shape(Object, [create, Object, ShapeTerm]) :-
    kb_call(object_shape(Object, _Frame, ShapeTerm, _Pose, _Material)),
    !.

%% giskard_test is det.
%
% giskard_test is a simple, manual test for giskard_update/3.
giskard_test :-
    create_object(_Object, 'http://www.ease-crc.org/ont/SOMA.owl#Shelf', [map, [0,0,0], [0,0,0,1]]),
    ros_info(giskard_test1),
    giskard_updates(0, Curr, Data),
    ros_info(giskard_test2),
    ros_info(Curr),
    ros_info(Data).
