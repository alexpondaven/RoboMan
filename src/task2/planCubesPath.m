function [via_paths,isBringCube] = planCubesPath(initCubesState, goalCubesState)
% planCubesPath     Determine via points to move cubes in initCubeState to
% desired goalCubeState
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
    [src_x, src_y, src_thetaG] = getHolderCoord(srcPos);
    [dst_x, dst_y, dst_thetaG] = getHolderCoord(dstPos);

    % Get height above cubes
    src_z = HEIGHT_OFFSET + (srcStack + 1) * CUBE_SIZE;
    dst_z = HEIGHT_OFFSET + (dstStack + 1) * CUBE_SIZE;

    % TODO: Experiment if we need to add offset to z depending on orientation grabbed?

    % Get list of positions (x,y,z,theta_g, whether to bring cube) to go to
    positions = [];

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
            src_thetaG = 0;
            dst_thetaG = -pi/2;
            % Are there destination positions where we cannot be at -pi/2?
            % - In this case we'd need to rotate in place rather than while moving
            % - Can check if destination position can be reached using IK2
            
            % Check if destination position cannot be reached with thetaG = -pi/2
            [_,ec] = inverseKin2(dst_x, dst_y, dst_z, dst_thetaG, false)
            if ec ~= 0
                % Rotate in-place
                positions = [positions;
                            [src_x, src_y, src_z, src_thetaG, false]; % Go to position without holding cube
                            [src_x, src_y, src_z, dst_thetaG, true]] % Rotate in place (first picking up cube)
            end

        case -1i
            % Rotate away from arm while translating?
            src_thetaG = -pi/2;
            dst_thetaG = 0;

            % Check if destination position cannot be reached with thetaG = -pi/2
            [_,ec] = inverseKin2(dst_x, dst_y, dst_z, dst_thetaG, false)
            if ec ~= 0
                % Rotate in-place
                positions = [positions;
                            [src_x, src_y, src_z, src_thetaG, false];
                            [src_x, src_y, src_z, dst_thetaG, true]]
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
    
    

    % Get via points between positions
    vias = [];
    for pos=2:size(positions,2)
        isBringCube(end+1) = positions(pos,5);
        % Get path between positions
        viaCoords = Asearch(positions(pos-1,1:4), positions(pos,1:4));

        % Convert to joint angles
        inverseKinRes = inverseKinDynamixel2( viaCoords(i,1), viaCoords(i,2), viaCoords(i,3), viaCoords(i,4), positions(pos,5));

        vias = [vias; inverseKinRes];
    end
    via_paths(end+1) = {};



end

end