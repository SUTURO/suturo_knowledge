%% urdf module
%  This module contains anything related to urdf.
:- module(urdf,
	  [
        get_urdf_id(-),
        get_urdf_origin(-),
        has_urdf_name(?,?),
        has_tf_name(?,?)
	  ]).

%% get_urdf_id(-URDF) is det.
%
% Gets the urdf id of the current environment
%
% @param URDF The urdf id
%
get_urdf_id(URDF) :-
    URDF = arena.

%% get_urdf_origin(-Origin) is det.
%
% Gets the urdf origin of the current environment
%
% @param Origin The urdf origin
%
get_urdf_origin(Origin) :-
    Origin = map.

%% has_urdf_name(?Object, ?URDFName) is nondet.
%
% Looks up which object has which urdfname
% works in both directions
%
% part of the planning interface.
% the api should stay stable.
has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).


%% has_tf_name(?Object, ?TFName) is nondet.
%
% Gets the tf name of an object.
% For objects that have a urdf name, the TFName is based on the urdf name.
% For other objects, it is the part after the #
%
% @param Object The object
% @param TFName The tf name of the object
%
has_tf_name(Object, TFName) :-
    % anything with a # is an object and not a urdf name
    sub_string(Object, _, _, After, "#"),
    (has_urdf_name(Object, URDFName) 
    -> 	has_tf_name(URDFName, TFName)
    % for stuff that doesn't have a urdf name, use the last part of the iri
    ; sub_atom(Object, _, After, 0, TFName)
    ),
    !.

%% has_tf_name(?Object, ?TFName) is semidet.
%
% Gets the tf name of an object that has a urdf name
%
% @param URDFName The urdf name of the object
% @param TFName The tf name of the object
%
has_tf_name(URDFName, TFName) :-
    % using a not here so the cut 3 lines above is a green cut.
    not(sub_string(URDFName, _, _, _, "#")),
    % TODO don't hardcode iai_kitchen
    atom_concat('iai_kitchen/', URDFName, TFName).
