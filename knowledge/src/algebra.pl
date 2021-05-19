:- module(algebra,
    [
        euclidean_distance/3,
        angle_to_quaternion/2
    ]).


euclidean_distance([X1, Y1, Z1], [X2, Y2, Z2], Distance) :-
    XDiff is X1 - X2, YDiff is Y1 - Y2, ZDiff is Z1 - Z2,
    XSquare is XDiff * XDiff,
    YSquare is YDiff * YDiff,
    ZSquare is ZDiff * ZDiff,
    Sum is XSquare + YSquare + ZSquare,
    Distance is sqrt(Sum).


angle_to_quaternion(Angle, Quaternion) :-
    C is cos(Angle/2),
    S is sin(Angle/2),
    Quaternion = [0.0, 0.0, S, C].