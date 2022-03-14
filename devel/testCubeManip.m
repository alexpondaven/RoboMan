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

    servoLimits = getServoLimits();
    velocityLimit = getDXLSettings().velocityLimit;

    % Initialize occupancy grid
    cube_locs = [ [3,-8, 0]; [9, 0, 0]; [6, 6, 0] ].';
    % We take the transpose to be able to index properly
    % Grid i,j coordinates, as well as current height
    % (in stacking terms) of the cube
    cube_hold = [ [3,-8]; [5,-5]; [4, 0]; [9, 0]; [0, 4]; [6, 6] ].';

    occupancyGrid = createOccupancyGrid(cube_locs, cube_hold);

    % Phase 1: Move from current position to startPos
    % TODO

    % Phase 2: Grip cube (move from startPos (open) -> cubePos (open) -> cubePos(closed) -> startPos(closed))
    % TODO
    
    % Phase 3: Move from startPos (closed) to endPos (closed)
    % Try to rotate cube at (9,0)
    
    curr_pos = zeros(1,4);
    for i=1:4
        curr_pos(i) = read4ByteTxRx(port_num, params.PROTOCOL_VERSION, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION);
    end
    
    currEndpointCoords = getCurrEndpointCoords(curr_pos);

    AStarWaypoints = [currEndpointCoords, startPos, endPos];
    via_paths = calcViaPoints(AStarWaypoints, occupancyGridVect);   % TODO define occupancyGridVect

    coeff_paths = interpViaPoints(via_paths, true);

    startPos = [100, 0, 50, -pi/2];
    endPos = [225, 0, 50, 0];

    GRIP_POS = deg2rad(232);

    % set isPlot to false for production
    if mainServoLoop(coeffs, T, Tend, port_num, true) ~= 0
        % ? Refactor such that it will GOTO END
    end

    % Phase 4: Deposit cube (move from endPos (closed) -> cubePos (closed) -> cubePos(open) -> endPos(open))

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