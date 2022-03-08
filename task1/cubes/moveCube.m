function moveCube(src,dst,state)
% MOVECUBE  Returns path to move cube at src to dst
%   If there are already cubes, pick the top one
%
% ARGS
%   src     cubeholder position to move cube from
%   dst     cubeholder position to move cube to
%   state   number of cubes in each position
%           e.g. cube at pos1 : state=(1,0,0,0,0,0)

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



end