function via_paths = planCubesPath(cubeMoves, cubeStacks)
% planCubesPath     Determine via paths for cube movements
%  Note: Just includes movements involving cubes, so extra via paths to go
%  to next start position must be added for full path planning.
%
% Args
% cubeMoves : array of cube movements
%          cube move = [src cube holder, dst cube holder, rotation]
%           cube rotation:
%           - 0 : no rotation
%           - -1: towards robot
%           - 1: away from robot
%   cubestates in [cubeholder, height, red face location]
%   e.g. {[1,1,"up"],[2,1,"back"]}
%   -> {[1,1,"up"],[2,1,"front"]}
%   -> {[2,2,"front"],[2,1,"front"]}
% 
%   cubeMoves = [[ 2,2,1]; % Rotate cube at 2 away from arm
%                [ 2,2,1]; % Rotate cube at 2 away from arm
%                [ 1,2,1]] % Move 1 to 2 (height 2) while
%               rotating away from arm
% 
%
%   Note: assume path is possible
%   Extension: Generate entire path automatically - requires a lot of checks,
%   that can easily lead to non-optimal path taken (this is safer)
% cubeStacks : Heights of cube stacks indexed by holder number 
%   e.g. [0,2,0,1,0,0] is 1 cube on holder 4, and 2 cubes on holder 2
%
% Return
% via_paths      : cell array of via points for each path
% curOccupancyGrid?

CUBE_SIZE = 25;
HEIGHT_OFFSET = 40; % Position above cube that can be reached in occupancy grid

currCubeStacks = cubeStacks;
via_paths = {};
occupancyGrid_paths
% Determine vias for every cube movement path
for i=1:size(cubeMoves,1)
    srcPos = cubeMoves(i,1);
    srcHeight = currCubeStacks(srcPos);
    dstPos = cubeMoves(i,2);
    dstHeight = currCubeStacks(dstPos);
    rotate = cubeMoves(i,3);

    % Get x,y coordinates of source and destination positions
    % TODO: remove src_thetaG and dst_thetaG from getHolderCoord as it just
    % assignes thetaG based on position rather than required rotation
    [src_x, src_y, src_thetaG] = getHolderCoord(srcPos);
    [dst_x, dst_y, dst_thetaG] = getHolderCoord(dstPos);

    % Get height above cubes
    src_z = HEIGHT_OFFSET + srcHeight * CUBE_SIZE;
    dst_z = HEIGHT_OFFSET + dstHeight * CUBE_SIZE;

    % TODO: Experiment if we need to add offset to z depending on orientation grabbed?

    % Rotate away involves theta_g = 0 -> theta_g = -pi/2
    % Rotate towards involves theta_g = -pi/2 -> theta_g = 0
    switch rotate
        case 0 % No rotation
            src_thetaG = 0;
            dst_thetaG = 0;
        case 1 % Rotate away from robot
            src_thetaG = 0;
            dst_thetaG = -pi/2;
        case -1 % Rotate towards robot
            src_thetaG = -pi/2;
            dst_thetaG = 0;
        otherwise
            disp("Not a valid rotation for cube movement")
    end

    % Generate occupancy grid
    % Convert cubestacks to input to createOccupancyGrid
    occupancyGrid = createOccupancyGrid(curr_cube_locs, cube_hold);

    % Update cube locations after computing occupancy grid
    currCubeStacks(srcPos) = currCubeStacks(srcPos) - 1;
    currCubeStacks(dstPos) = currCubeStacks(dstPos) + 1;


    % Calculate via points for path
    waypoints = [[src_x, src_y, src_z, src_thetaG];
                 [dst_x, dst_y, dst_z, dst_thetaG]];
    vias = calcViaPoints(waypoints, occupancyGrid);
    via_paths(end+1) = vias;


end

end