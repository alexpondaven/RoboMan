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

index = 1;
dxl_comm_result = params.COMM_TX_FAIL;           % Communication result
dxl_goal_position = [params.DXL_MINIMUM_POSITION_VALUE params.DXL_MAXIMUM_POSITION_VALUE];         % Goal position

dxl_error = 0;                              % Dynamixel error
dxl_present_position = 0;                   % Present position


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

initDynamixels(port_num);


%% DO STUFF HERE
setServoMode('vel', port_num);
% for i=1:100
writeToServos([10 0 0 0 0], 'vel', port_num);
% end
pause(1)
writeToServos([0 0 0 0 0], 'vel', port_num);


%% -- Dynamixel Cleanup Start -- %%
for i=1:length(params.DXL_LIST)
    % Disable Dynamixel Torque
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
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

% close all;
clear