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

og = createOccupancyGrid(cube_locs, cube_hold);

disp('Created occupancy grid.')

% TODO fix bugs in reachability analysis!!!

startPos = [ 100, -100, 50, -pi/2 ];
endPos = [ 200, 100, 50, 0 ];

waypoints = AstarSearch( startPos, endPos, og );
waypoints = [startPos; waypoints; endPos];    % add in the start and end poses!

posPath = zeros([ size(waypoints,1) ,5]);
for i=1:size(waypoints,1)
    posPath(i,:) = inverseKin2(waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), true);
end

visualisePath(waypoints, posPath)

% TODO: More robustly test A* driver code especially on manipulating cube