%% The Util module contains predicates that (currently) don't fit into other modules.
:- module(util,
	  [
	      has_urdf_name/2,
	      has_tf_name/2,
	      ros_info/2,
	      ros_warn/2,
	      ros_error/2,
	      ros_debug/2
	  ]).

:- use_module(library('lang/terms/triple'),
	      [
		  triple/3
	      ]).

%% has_urdf_name(?Object, ?URDFName) is nondet.
%
% Looks up which object has which urdfname
% works in both directions
%
% part of the planning interface.
% the api should stay stable.
has_urdf_name(Object, URDFName) ?+>
    triple(Object, urdf:'hasURDFName', URDFName).

%% has_tf_name(Object, TFName)
%
% gets the tf name of an object.
% for objects that have a urdf name, the TFName is based on the urdf name.
% for other objects, it is the part after the #
%
% part of the planning interface.
% the api should stay stable.
has_tf_name(Object, TFName) :-
    % anything with a # is an object and not a urdf name
    sub_string(Object, _, _, After, "#"),
    (
	has_urdf_name(Object, URDFName) ->
	has_tf_name(URDFName, TFName);
	% for stuff that doesn't have a urdf name, use the last part of the iri
	sub_atom(Object, _, After, 0, TFName)
    ),
    !.

% has_tf_name for urdf names
has_tf_name(URDFName, TFName) :-
    % using a not here so the cut 3 lines above is a green cut.
    not(sub_string(URDFName, _, _, _, "#")),
    % TODO don't hardcode iai_kitchen
    atom_concat('iai_kitchen/', URDFName, TFName).

%% ros_debug(+Format, +Arguments)
ros_debug(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_debug(MSG).

%% ros_info(+Format, +Arguments)
ros_info(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_info(MSG).

%% ros_warn(+Format, +Arguments)
ros_warn(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_warn(MSG).

%% ros_error(+Format, +Arguments)
%
% Logs an error message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_error(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_error(MSG).
