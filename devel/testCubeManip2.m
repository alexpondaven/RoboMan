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
    [notfound, warnings] = loadlibrary(lib_name, 'dynamixel_sdk.h', 'addheader', 'port_handler.h', 'addheader', 'packet_handler.h');
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
    GRAB_DEPTH = 10; % TUNE : How far to descend when grabbing/picking cube
    isPlot = true;

    % Setup
    servoLimits = getServoLimits();
    velocityLimit = getDXLSettings().velocityLimit;
    
    curr_pos = zeros(1,4);
    for i=1:4
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    end
    currEndpointCoords = getCurrEndpointCoords(curr_pos);

    % Cube movements [src cube holder, dst cube holder, rotation]
    %           rotation:
    %           - 0 : no rotation
    %           - -1: towards robot
    %           - 1: away from robot
    %   Cube states defines state of cubes (just for us to write state down easily): 
    %               [cubeholder, height, red face location]
    %                e.g. {[1,1,"up"],[2,1,"back"]} describes cube on holder1 facing up and cube on holder2 facing back (towards robot)
    cubeMoves = [[2,2,1]; % Rotate cube at 2 away from arm
                [2,2,1]; % Rotate cube at 2 away from arm
                [1,2,1]];
    cubeStacks = [1,1,0,0,0,0];
    
    % Get vias for cube movement
    [cube_via_paths, path_isholdingcube, waypoints] = planCubesPath(cubeMoves, cubeStacks, currEndpointCoords);
    % Interpolate between via points
    [coeff_paths, T_paths, Tend_paths] = interpViaPoints(cube_via_paths(i), isPlot);

    % For each path
    % - Check if path is holding cube
    % - If moving cube:
    %   - Grab cube
    %   - Follow path
    %   - Drop cube
    % - Otherwise, just move path

    % Movement code
    for i=1:size(path_isholdingcube,1)
        if path_isholdingcube{i}
            path_waypoints = waypoints{i};
            startPos = path_waypoints(1,:);
            endPos = path_waypoints(2,:);
            
            % Grab cube
            if cubePickPlace(startPos, startPos - GRAB_DEPTH, startPos, true, port_num) ~= 0

            end
            
            % Follow trajectory
            if mainServoLoop(coeff_paths(i), T_paths(i), Tend_paths(i), port_num, isPlot) ~= 0

            end

            % Drop cube
            if cubePickPlace(endPos, endPos - GRAB_DEPTH, endPos, false, port_num) ~= 0
            
            end

        else
            % Follow trajectory
            if mainServoLoop(coeff_paths(i), T_paths(i), Tend_paths(i), port_num, isPlot) ~= 0

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