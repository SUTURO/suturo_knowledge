%% The Util module contains predicates that (currently) don't fit into other modules.
:- module(util,
	  [
	      has_urdf_name(?,?),
	      has_tf_name(?,?),
          split_iri(+, -, -),
	      ros_info(+,+),
	      ros_warn(+,+),
	      ros_error(+,+),
	      ros_debug(+,+)
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

%% has_tf_name(?Object, ?TFName) is nondet.
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

%% has_tf_name(?Object, ?TFName) is semidet.
%
% gets the tf name of an object that has a urdf name
%
has_tf_name(URDFName, TFName) :-
    % using a not here so the cut 3 lines above is a green cut.
    not(sub_string(URDFName, _, _, _, "#")),
    % TODO don't hardcode iai_kitchen
    atom_concat('iai_kitchen/', URDFName, TFName).


% split_iri(+IRI, -Prefix, -ClassIdentifier)
% 
% Splits an IRI of the form <Prefix>#<ClassIdentifier> into the prefix part and the class identifier.
% 
% @param IRI The IRI to split
% @param Prefix The prefix part of the IRI
% @param ClassIdentifier The class identifier part of the IRI
% 
split_iri(IRI, Prefix, ClassIdentifier) :-
    sub_atom(IRI, Before, _, After, '#'),
    !,
    sub_atom(IRI, 0, Before, _, Prefix),
    sub_atom(IRI, _, After, 0, ClassIdentifier).


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Debugging
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% ros_debug(+Format, +Arguments)
%
% Logs a debug message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_debug(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_debug(MSG).

%% ros_info(+Format, +Arguments)
%
% Logs an info message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
ros_info(Format, Arguments) :-
    format(string(MSG), Format, Arguments),
    ros_info(MSG).

%% ros_warn(+Format, +Arguments)
%
% Logs a warning message and formats with the given arguments.
%
% @param Format The format string
% @param Arguments The arguments to the format string
%
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
