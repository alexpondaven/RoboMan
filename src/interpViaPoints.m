function [coeff_paths, T_paths, Tend_paths] = interpViaPoints(via_paths, isPlot)
% interpViaPoints  Interpolate using quintic between via points and return coeffs
%
% Args
% via_paths     : Cell array for via points for each path (each start and end point has several via points comprising a path)
% isPlot        : Whether to plot quintic interpolation
%
% Return
% coeff_paths   : Cell array of coefficients for each path
% T_paths       : Time of each via point (should be same #rows as vias) - for each path
% Tend_paths    : Time to reach last via point - for each path

    coeff_paths = {};
    T_paths = {};
    Tend_paths = {};
    for i=1:size(via_paths,2)
        % Start with current position? - but we don't know current position after every segment
        % Assume at correct position at start of segment after mainServoLoop is run
        % vias(1,:) = curr_pos;

        % Interpolate between waypoints
        vias = via_paths{i}
        
        if size(vias,1)==0
            coeffs = [];
            T = [];
            Tend = 0;
        else
            [T, Tend] = assignViaTimes(vias, 'lin');    % Tend no longer used
            coeffs = interpQuinticTraj(vias, T);
            
            if isPlot
                figure
                plotQuinticInterp(vias, coeffs, T);
            end
        end
            
        coeff_paths(end+1) = {coeffs};
        T_paths(end+1) = {T};
        Tend_paths(end+1) = {Tend};
    end

end