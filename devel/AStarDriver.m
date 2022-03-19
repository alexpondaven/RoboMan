% Define occupancy grid

% Theta_g range is [-pi/2, pi/2]. We can set 10 degree increments

% Wrap this into a function in the future that takes a list of cube and
% cube holder locations (i,j)

clear
clc

cube_locs = [
    [3,-8, 0]
    [9, 0, 0]
    [6, 6, 0]
].';                     % We take the transpose to be able to index properly
                        % Grid i,j coordinates, as well as current height
                        % (in stacking terms) of the cube

cube_hold = [
    [3,-8]
    [5,-5]
    [4, 0]
    [9, 0]
    [0, 4]
    [6, 6]
].';

THETA_1_LIST = getOccupancyGridParams().THETA_1_LIST;
ARM_END = getArmDimensions().L4;

t1 = now;
og = createOccupancyGrid([1,1,1,0,0,0]);
t2 = now;
t_og = ((t2-t1)*24)*3600;

disp('Created occupancy grid.')

% TODO fix bugs in reachability analysis!!!

startPos = [ 75, -200, 50, 0 ];
endPos =   [ 125, -125, 50, -pi/2 ];

% startPos = [ 145, 50, 75, 0 ];
% endPos =   [ 145, 50, 45, -pi/2 ];

t1 = now;
waypoints = AstarSearch( startPos, endPos, og );
t2 = now;
t_astar = ((t2-t1)*24)*3600;

% Longer paths can take up to a minute or more to generate. Be patient.

fprintf("Took %0.2fs to generate occupancy grid and %0.2fs to perform A* search.\n\n", t_og, t_astar);

waypoints = [startPos; waypoints; endPos];    % add in the start and end poses!

posPath = zeros([ size(waypoints,1) ,5]);
for i=1:size(waypoints,1)
    posPath(i,:) = inverseKin2(waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), true);
end

visualisePath(waypoints, posPath)

%% String it together with interpolation
Tend = size(waypoints, 1);  % on average give 1 second per waypoint?
[coeffs, T] = interpTraj(vias,Tend)
plotInterp(vias, coeffs, T)