%% Driver function to take advantage of group read and writes.

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

    % Set up. We should remove this and put this into its own function later.
    COMM_SUCCESS = 0;
    groupBulkReadNum = groupBulkRead(port_num, params.PROTOCOL_VERSION);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_POSITION, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(2), params.ADDR_PRO_PRESENT_POSITION, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(3), params.ADDR_PRO_PRESENT_POSITION, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(4), params.ADDR_PRO_PRESENT_POSITION, 4);
    
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(1), params.ADDR_PRO_PRESENT_VELOCITY, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(2), params.ADDR_PRO_PRESENT_VELOCITY, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(3), params.ADDR_PRO_PRESENT_VELOCITY, 4);
    groupBulkReadAddParam(groupBulkReadNum, params.DXL_LIST(4), params.ADDR_PRO_PRESENT_VELOCITY, 4);

    % Read present position and velocity
    groupBulkReadTxRxPacket(groupBulkReadNum);
    % Check for errors
    if getLastTxRxResult(port_num, params.PROTOCOL_VERSION) ~= COMM_SUCCESS
        % printTxRxResult(params.PROTOCOL_VERSION, getLastTxRxResult(port_num, params.PROTOCOL_VERSION));
        disp("Error in bulk read");
    end

    curr_pos = zeros(1,4);
    curr_vel = zeros(1,4);
    
    % Readout values
    for i=1:4
        curr_pos(i) = groupBulkReadGetData(groupBulkReadNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_POSITION, 4);
        curr_vel(i) = twos2decimal(groupBulkReadGetData(groupBulkReadNum, params.DXL_LIST(i), params.ADDR_PRO_PRESENT_VELOCITY, 4), 32);
    end
    
    currEndpointCoords = getCurrEndpointCoords(curr_pos);

    % Simple moving in a square as usual
    z = 100;
    square = [
        175  100 z;
        175 -100 z;
        175  100 z;
        % 75  50  z;
        % 75 -50  z; 
    ];
    % square(end+1,:) = square(1,:);  % make it complete the square

    % interpolate lines between corners
    corners = [];
    numPoints = 40;
    for i=2:size(square, 1)
        corners = [corners; linearInterpolate(square(i-1,:), square(i,:), numPoints) ];
    end
    
    % do inverse kinematics
    vias = zeros(length(corners), 4);
    for i=1:length(corners)
        theta = inverseKinDynamixel2(corners(i,1), corners(i,2), corners(i,3), -pi/2, true);
        vias(i,:) = theta(1:4);
    end

    % Interpolate and assign timings
    [T, Tend] = assignViaTimes(vias, 'lin');
    coeffs = interpQuinticTraj(vias, T);
    figure
    plotQuinticInterp(vias, coeffs, T);

    mainServoLoop2(coeffs, T, Tend, port_num, true, vias);
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