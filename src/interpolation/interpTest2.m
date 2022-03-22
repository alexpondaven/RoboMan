vias = [0 0 0 0 5
        1 4 9 1 4
        2 5 2 2 3
        8 2 4 6 0];
% Tend =15;

[T, Tend] = assignViaTimes(vias, 'lin');    % Tend no longer used
coeffs = interpQuinticTraj(vias, T);
plotQuinticInterp(vias,coeffs, T)