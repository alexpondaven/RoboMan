%% Code to test velocity control

%% Initialization
clc;
clear;

lib_name = '';

if strcmp(computer, 'PCWIN')
  lib_name = 'dxl_x86_c';
elseif strcmp(computer, 'PCWIN64')
  lib_name = 'dxl_x64_c';
elseif strcmp(computer, 'GLNX86')
  lib_name = 'libdxl_x86_c';
elseif strcmp(computer, 'GLNXA64')
  lib_name = 'libdxl_x64_c';
elseif strcmp(computer, 'MACI64')
  lib_name = 'libdxl_mac_c';
end

% Load Libraries
if ~libisloaded(lib_name)
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h', ...
    'addheader', 'group_sync_write.h', 'addheader', 'group_sync_read.h', 'addheader', 'group_bulk_read.h', 'addheader', 'group_bulk_write.h');
end

params = getDXLParams();
port_num = portHandler(params.DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();
dxl_comm_result = params.COMM_TX_FAIL;           % Communication result
dxl_error = 0;                              % Dynamixel error

% Open port
if (openPort(port_num))
    fprintf('Port Open\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to open the port\n');
    input('Press any key to terminate...\n');
    return;
end

% Set port baudrate
if (setBaudRate(port_num, params.BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Check for error in initializing dynamixels
if initDynamixels(port_num, 'vel') == 0
    %% DO STUFF HERE

    % CONSTANTS
    ogParams = getOccupancyGridParams();
    GRAB_DEPTH = ogParams.HOVER_HEIGHT + ogParams.CUBE_DIM/2; % How far to descend when grabbing/picking cube
    isPlot = true;

    % Setup
    servoLimits = getServoLimits();
    velocityLimit = getDXLSettings().velocityLimit;
    
    curr_pos = zeros(1,4);
    for i=1:4
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    end
    fprintf("curr_pos: %04d | %04d | %04d | %04d\n", curr_pos(1), curr_pos(2), curr_pos(3), curr_pos(4));
    currEndpointCoords = getCurrEndpointCoords(curr_pos);

    fprintf("currEndpointCoords: %04d | %04d | %04d | %04d\n", currEndpointCoords(1), currEndpointCoords(2), currEndpointCoords(3), currEndpointCoords(4));
    % SET CUBE MOVEMENTS
    % Format:  [src cube holder, dst cube holder, rotation]
    %           rotation:
    %           - 0 : no rotation
    %           - -1: towards robot
    %           - 1: away from robot
    %   Cube states defines state of cubes (just for us to write state down easily): 
    %               [cubeholder, height, red face location]
    %                e.g. {[1,1,"up"],[2,1,"back"]} describes cube on holder1 facing up and cube on holder2 facing back (towards robot)

    % Video setup: {[1,1,"away"],[2,1,"down"],[3,1,"away"]}
    % Task 2a Translation - from start to arbitrary finish location
    cubeMoves = [[1,4,0]
                 [2,5,0]
                 [3,6,0]];
    cubeStacks = [1,1,1,0,0,0];


    % Task 2b Rotation inplace - so red face is at top
    cubeMoves = [[1,1,-1]
                 [2,2,1]
                 [2,2,1]
                 [3,3,-1]];
    cubeStacks = [1,1,1,0,0,0];

    % Task 3b Stacking - stack in any finishing location (4?), with all red
    % faces facing away
    cubeMoves = [[1,4,0]
                 [2,4,1]
                 [3,4,0]];
    cubeStacks = [1,1,1,0,0,0];
    
    % Test movements
    cubeMoves = [3,3,1];
    cubeStacks = [0,1,0,0,0,0];
    
    % Get vias for cube movement
    [cube_via_paths, path_isholdingcube, waypoints] = planCubesPath(cubeMoves, cubeStacks, currEndpointCoords);
    % Interpolate between via points
    [coeff_paths, T_paths, Tend_paths] = interpViaPoints(cube_via_paths, isPlot);

    % For each path
    % - Check if path is holding cube
    % - If moving cube:
    %   - Grab cube
    %   - Follow path
    %   - Drop cube
    % - Otherwise, just move path

    % Movement code
    for i=1:size(path_isholdingcube,2)
        if Tend_paths{i} <= 0
            disp("[testCubeManip2] No move necessary");
            continue
        else
            if path_isholdingcube(i)
                path_waypoints = waypoints{i};
                startPos = path_waypoints(1,:);
                endPos = path_waypoints(2,:);
                
                % Grab cube
                disp("[testCubeManip2] Grab cube");
                if cubePickPlace(startPos, startPos - [0,0,GRAB_DEPTH,0], startPos, true, port_num) ~= 0
                    disp("[testCubeManip2] Grab cube failed")
                    return
                end
                
                % Follow trajectory
                disp("[testCubeManip2] Move cube");
                if mainServoLoop2(coeff_paths{i}, T_paths{i}, Tend_paths{i}, port_num, isPlot, cube_via_paths{i}) ~= 0
                    disp("[testCubeManip2] Move cube failed")
                    return
                end

                % Drop cube
                disp("[testCubeManip2] Drop cube");
                % Smaller depth for dropping 
                if cubePickPlace(endPos, endPos - [0,0,GRAB_DEPTH-20,0], endPos, false, port_num) ~= 0
                    disp("[testCubeManip2] Drop cube failed")
                    return
                end

            else
                % Follow trajectory
                disp("[testCubeManip2] Move arm");
                if mainServoLoop2(coeff_paths{i}, T_paths{i}, Tend_paths{i}, port_num, isPlot, cube_via_paths{i}) ~= 0
                    disp("[testCubeManip2] Move arm failed")
                    return
                end
            end
        end
    end


end

%% -- Dynamixel Cleanup Start -- %%
for i=1:length(params.DXL_LIST)
    % Disable Dynamixel Torque
    % write1ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_TORQUE_ENABLE, 0);
    dxl_comm_result = getLastTxRxResult(port_num, params.PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, params.PROTOCOL_VERSION);
    if dxl_comm_result ~= params.COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(params.PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(params.PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);