function coeff_paths = interpViaPoints(via_paths, isPlot)
% interpViaPoints  Interpolate using quintic between via points and return coeffs
%
% Args
% via_paths     : Cell array for via points for each path (each start and end point has several via points comprising a path)
% isPlot        : Whether to plot quintic interpolation
%
% Return
% coeff_paths   : Cell array of coefficients for each path
    coeff_paths = {};
    for i=1:size(via_paths,1)
        % Start with current position? - but we don't know current position after every segment
        % Assume at correct position at start of segment after mainServoLoop is run
        % vias(1,:) = curr_pos;

        % Interpolate between waypoints
        vias = via_paths(i)
        [T, Tend] = assignViaTimes(vias, 'dvel');    % Tend no longer used
        coeffs = interpQuinticTraj(vias, T);

        if isPlot
            figure
            plotQuinticInterp(vias, coeffs, T);
        end

        coeff_paths(end+1) = {coeffs};
    end

end