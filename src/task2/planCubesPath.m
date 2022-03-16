function [via_paths,isBringCube] = planCubesPath(initCubesState, goalCubesState)
% planCubesPath     Determine via points to move cubes in initCubeState to
% desired goalCubeState.
%
% Each via path has corresponding boolean whether to bring cube
% - If bring cube, first pick up, go through path, then drop cube
%
% Args
% initCubesState : array of initial cube states
%          [cube holder number, cube stack height, cube orientation]
%           cube orientation:
%           - "up": upwards
%           - "back": towards robot
%           - "front": away from robot
%           - "down": downwards
% goalCubesState : array of final cube states
%
% Return
% via_paths      : cell array of via points for each path
% bring_cube     : list of whether to carry cubes

CUBE_SIZE = 25;
HEIGHT_OFFSET = 40; % Position above cube that can be reached in occupancy grid

via_paths = {};
isBringCube = [];
% Plan trajectory for every cube
for i=1:size(initState,1)
    srcPos = initCubesState(i,1);
    srcStack = initCubesState(i,2);
    srcOrient = initCubesState(i,3);
    dstPos = goalCubesState(i,1);
    dstStack = goalCubesState(i,2);
    dstOrient = goalCubesState(i,3);

    % Get x,y coordinates of source and destination positions
    % TODO: remove src_thetaG and dst_thetaG from getHolderCoord as it just
    % assignes thetaG based on position rather than required rotation
    [src_x, src_y, src_thetaG] = getHolderCoord(srcPos);
    [dst_x, dst_y, dst_thetaG] = getHolderCoord(dstPos);

    % Get height above cubes
    src_z = HEIGHT_OFFSET + srcStack * CUBE_SIZE;
    dst_z = HEIGHT_OFFSET + dstStack * CUBE_SIZE;

    % TODO: Experiment if we need to add offset to z depending on orientation grabbed?


    vias = [];

    % Determine if rotate away or towards bot
    % Rotate away involves theta_g = 0 -> theta_g = -pi/2
    % Rotate towards involves theta_g = -pi/2 -> theta_g = 0
    % If rotating 180 degrees, just rotate away twice

    % Express orientation as imaginary number
    srcOrientImag = orientToImag(srcOrient);
    dstOrientImag = orientToImag(dstOrient);
    
    switch srcOrientImag / dstOrientImag
        case 1i
            % Rotate towards arm while translating?
            src_thetaG = -pi/2;
            dst_thetaG = 0;
            
            % src is always on cubeholder, so can pick up with thetaG=-pi/2
            [_,ec] = inverseKin2(dst_x, dst_y, dst_z, dst_thetaG, false);
            if ec ~= 0
                % Rotate in-place
                viasRotate = AstarSearch([src_x, src_y, src_z, src_thetaG], [src_x, src_y, src_z, dst_thetaG])
                vias = [vias; viasRotate]
                isBringCube(end+1) = true;
            end

        case -1i
            % Rotate away from arm while translating?
            src_thetaG = 0;
            dst_thetaG = -pi/2;
            % Assume all positions can be grabbed with thetaG=0

            % Check if destination position cannot be reached with thetaG = -pi/2
            [_,ec] = inverseKin2(dst_x, dst_y, dst_z, dst_thetaG, false);
            if ec ~= 0
                % Rotate in-place
                viasRotate = AstarSearch([src_x, src_y, src_z, src_thetaG], [src_x, src_y, src_z, dst_thetaG])
                vias = [vias; viasRotate]
                isBringCube(end+1) = true;
            end

        case -1
            % Rotate 180 degrees (rotate away from arm twice)

            % First add rotation motion to positions

            % Then set up thetaGs to change when translating

        case 1
            % No rotation required
        
        otherwise
            disp("u wut m8")

    end

    % Determine where to translate cube
    
    

    % Convert via coordinates to joint angles
    
    for pos=2:size(positions,2)
        isBringCube(end+1) = positions(pos,5);
        % Get path between positions
        viaCoords = AstarSearch(positions(pos-1,1:4), positions(pos,1:4));

        % Convert to joint angles
        inverseKinRes = inverseKinDynamixel2( viaCoords(i,1), viaCoords(i,2), viaCoords(i,3), viaCoords(i,4), positions(pos,5));

        vias = [vias; inverseKinRes];
    end
    via_paths(end+1) = {vias};



end

end