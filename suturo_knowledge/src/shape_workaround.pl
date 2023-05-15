%%
% A Module to workaround a bug with object_shape/5.
% See [knowrob#368](https://github.com/knowrob/knowrob/issues/368) for more information.
:- module(shape_workaround,
	  [object_shape_workaround(r,?,?,?,?)]
	 ).

:- use_module(library('model/SOMA'),
	      [object_shape/5]
	     ).


%% object_shape_workaround(+Obj, ?Frame, ?ShapeTerm, ?Pose, ?Material) is nondet.
%
% This is a workaround for [knowrob#368](https://github.com/knowrob/knowrob/issues/368).
% Note that this workaround can only return the first result of calling object_shape.
% for more information on the function it calls, see `object_shape/5`.
object_shape_workaround(Obj, Frame, ShapeTerm, Pose, Material) :-
    % as soon as this succeeds once, don't try again.
    once((member(_Try, [1,2,3]), % try up to three times
	  (
	      kb_call(object_shape(Obj, Frame, ShapeTerm, Pose, Material))
	  ->  true
	  % if the call fails, cut and don't try again
	  ;   !, false
	  ),
	  % if the ShapeTerm is not valid, fail so that prolog backtracks to the choice point at member.
	  is_valid_shape(ShapeTerm)
	 )).

is_valid_shape(ShapeTerm) :-
    compound(ShapeTerm),
    (
	ShapeTerm = box(_,_,_);
	ShapeTerm = mesh(_,[_,_,_]);
	ShapeTerm = sphere(_);
	ShapeTerm = cylinder(_,_)
    ),
    !.
