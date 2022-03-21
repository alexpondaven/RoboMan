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

    %% Main function for Task 4
    isPlot = true;
    % Generate path
    [waypoints, buzzAngle] = genBuzzPath();
    theta5 = arrayfun(buzzAngleToTheta5, buzzAngle);

    % IK - convert to joint angles
    theta_vias = zeros( size(waypoints, 1), 4 );
    for i=1:size(waypoints, 1)
        % Note: Gripper open/close not important here as only theta1..4 taken
        ikResult = inverseKinDynamixel2( waypoints(i,1), waypoints(i,2), waypoints(i,3), waypoints(i,4), true ); 
        theta_vias(i, :) = ikResult(1:4);
    end
    % Interpolation of joint angles for single path {theta_vias}
    [coeff_paths, T_paths, Tend_paths] = interpViaPoints({theta_vias}, isPlot);

    % Follow trajectory - should just be one path
    disp("[playBuzzWire] Move buzz wire");
    if mainServoLoop2(coeff_paths{1}, T_paths{1}, Tend_paths{1}, port_num, isPlot, cube_via_paths{i}) ~= 0
        disp("[playBuzzWire] Move buzz wire failed")
        return
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