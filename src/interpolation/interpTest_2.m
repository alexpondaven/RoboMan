vias = [0 0 0 0
        1 4 9 1
        2 0 2 2
        3 4 4 6
        4 0 4 6
        5 2 4 6
        6 2 4 6
        7 2 4 6];
% Tend =15;

[T, Tend] = assignViaTimes(vias, 'acc');    % Tend no longer used
coeffs = interpQuinticTraj(vias, T);
plotQuinticInterp(vias, coeffs, T)
