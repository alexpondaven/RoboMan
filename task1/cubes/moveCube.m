function pathPos = moveCube(src,dst,state)
% MOVECUBE  Returns path to move cube at src to dst
%   If there are already cubes, pick the top one
%
% ARGS
%   src     cubeholder position to move cube from
%   dst     cubeholder position to move cube to
%   state   number of cubes in each position
%           e.g. cube at pos1 : state=(1,0,0,0,0,0)
%
% OUT
%   pathPos     [x,y,z,thetaG,gripSize] for all positions

% POSITIONS OF ALL CUBES ON CUBEHOLDERS
% 1) grids=(3,-8) - from side
%   (75,-200,40,0,deg2rad(212))
% 2) grids=(9,0) -from side
%   (225,0,40,0,deg2rad(212))
% 3) grids=(6,6) - from side
%   (150,150,40,0,deg2rad(212))
% 4) grids=(5,-5) - from top
%   (125,-125,50,-pi/2,deg2rad(212))
% 5) grids=(4,0) - from top
%   (100,0,50,-pi/2,deg2rad(212))
% 6) grids = (0,4) - from top
%   (0,100,50,-pi/2,deg2rad(212))

% No rotation for now
%% Get positions

% Params
open_grip = deg2rad(95);
closed_grip = deg2rad(212);
holder_offset = 20; % How much to raise cube above placeholder
initial_height = 40; % height to pick up cube on placeholder from side
cube_height = 25;
startPos = [100,0,100,0,open_grip];

% Get x,y coord from src/dst number
    function [x,y,thetaG] = getHolderCoord(holderNum)
        switch holderNum
            case 1
                grid = [3,-8];
                thetaG = 0;
            case 2
                grid = [9,0];
                thetaG = 0;
            case 3
                grid = [6,6];
                thetaG = 0;
            case 4
                grid = [5,-5];
                thetaG = -pi/2;
            case 5
                grid = [4,0];
                thetaG = -pi/2;
            case 6
                grid = [0,4];
                thetaG = -pi/2;
            otherwise
                warning("Invalid holder number - only 6 cube holders exist")
        end
        coord = grid * cube_height;
        x = coord(1);
        y = coord(2);
    end

[src_x, src_y, src_thetaG] = getHolderCoord(src);
[dst_x, dst_y, dst_thetaG] = getHolderCoord(dst);

% Get z of placeholders (accounting for current state)

if src_thetaG == -pi/2
    gripper_offset = 10; % Account for increase height when picking from top (hack)
else
    gripper_offset = 0;
end

src_z = initial_height + cube_height * state(src) + gripper_offset;
dst_z = initial_height + cube_height * state(dst) + gripper_offset;

% Get max z from state to determine how high to move
max_height = initial_height + cube_height * max(state) + holder_offset;

%% Path planning

% Starting position
pathPos = startPos;

% Go to src 
pathPos = [pathPos; 
            [src_x,src_y,src_z,src_thetaG,open_grip]];

% Grasp cube
pathPos = [pathPos; 
            [src_x,src_y,src_z,src_thetaG,closed_grip]];

% Move straight up from src z (above placeholder)
pathPos = [pathPos; 
            [src_x,src_y,src_z + holder_offset,src_thetaG,closed_grip]];

% Move x' closer (halfway towards itself)
pathPos = [pathPos; 
            [src_x/2,src_y/2,src_z + holder_offset,src_thetaG,closed_grip]];

% Move to max z while rotating to dst_thetaG
pathPos = [pathPos; 
            [src_x/2,src_y/2,max_height,dst_thetaG,closed_grip]];

% Move above dst
pathPos = [pathPos; 
            [dst_x/2, dst_y/2 ,max_height,dst_thetaG,closed_grip]];

% Move down to dst z
pathPos = [pathPos; 
            [dst_x, dst_y ,dst_z,dst_thetaG,closed_grip]];

% Drop
pathPos = [pathPos; 
            [dst_x, dst_y , dst_z,dst_thetaG, open_grip]];

% Go to max height again
pathPos = [pathPos; 
            [dst_x, dst_y , dst_z,dst_thetaG, open_grip]];


%% Update state?

end