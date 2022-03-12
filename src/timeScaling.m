function dT = timeScaling(vias, Tend)
% TIMESCALING Heuristic for scaling the timestamps associated with each trajectory based on
% their percieved changes in velocity 
%
% Args:
% vias : Vector of 4 theta values that the trajectory must pass through
% Tend : Time to reach final via point
%
% Returns:
% dT : vector of timestamps that each trajectory passes through

dT = zeros( size(vias,1) );
dT(end) = Tend;

% TODO implement OneNote Task2>Trajectory interpolation

end