% Define occupancy grid

% Theta_g range is [-pi/2, pi/2]. We can set 10 degree increments

% Wrap this into a function in the future that takes a list of cube and
% cube holder locations (i,j)

clear

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

startPos = [ 50, -100, 50, -pi/2 ];
endPos = [ 100, 100, 100, -pi/4 ];

waypoints = AstarSearch( startPos, endPos, og );

% TODO: More robustly test A* driver code especially on manipulating cube

plot3([0,250], [0,  0], [0,  0], 'r', 'LineWidth', 1.0)
hold on
plot3([0,  0], [0,250], [0,  0], 'g', 'LineWidth', 1.0)
plot3([0,  0], [0,  0], [0,250], 'b', 'LineWidth', 1.0)
plot3(startPos(1), startPos(2), startPos(3), 'gx', 'LineWidth', 2.0)
plot3(endPos(1), endPos(2), endPos(3), 'rx', 'LineWidth', 2.0)
axis([-50, 250, -250, 250, 0, 250])

plot3(waypoints(:,1), waypoints(:,2), waypoints(:,3), 'kx-')

for i=1:size(waypoints,1)
    
    % Obtain transform of last joint (a bit hardcoded)
    base = [ [1 0 0 waypoints(i, 1)];
             [0 1 0 waypoints(i, 2)];
             [0 0 1 waypoints(i, 3)];
             [0 0 0 1              ] ];
    trf0 =  getDHTransform(0, 0, pi/2, 0);
    trf1 =  getDHTransform(pi + waypoints(i,4), 0, 0, 0);
    trf2 =  getDHTransform(0, ARM_END, 0, 0);
    loc = ((base*trf0)*trf1)*trf2;

    plot3([waypoints(i,1), loc(1,4)], [waypoints(i,2), loc(2,4)], [waypoints(i,3), loc(3,4)], 'm.--')

end

grid on

hold off

% for i=1:length(THETA_1_LIST)
%     fprintf("Occupancy grid for angle %d", THETA_1_LIST(i));
%     squeeze( og(1,i,:,:) )
% end
