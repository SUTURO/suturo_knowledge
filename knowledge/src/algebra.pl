:- module(algebra,
    [
        euclidean_distance/3
    ]).


euclidean_distance([X1, Y1, Z1], [X2, Y2, Z2], Distance) :-
    XDiff is X1 - X2, YDiff is Y1 - Y2, ZDiff is Z1 - Z2,
    XSquare is XDiff * XDiff,
    YSquare is YDiff * YDiff,
    ZSquare is ZDiff * ZDiff,
    Sum is XSquare + YSquare + ZSquare,
    Distance is sqrt(Sum).