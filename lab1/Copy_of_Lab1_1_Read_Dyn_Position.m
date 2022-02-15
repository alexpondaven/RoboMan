% Read the position of the dynamixel horn with the torque off
% The code executes for a given amount of time then terminates


clc;
clear all;

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

%% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;

%% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_ID                      = 13;            % Dynamixel ID: 1
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM3';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ------------------ %%

% Initialize PortHandler Structs
% Set the port path
% Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = portHandler(DEVICENAME);

% Initialize PacketHandler Structs
packetHandler();

index = 1;
dxl_comm_result = COMM_TX_FAIL;           % Communication result
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE DXL_MAXIMUM_POSITION_VALUE];         % Goal position

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
if (setBaudRate(port_num, BAUDRATE))
    fprintf('Baudrate Set\n');
else
    unloadlibrary(lib_name);
    fprintf('Failed to change the baudrate!\n');
    input('Press any key to terminate...\n');
    return;
end

% Put actuator into Position Control Mode
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_OPERATING_MODE, 3);


% Disable Dynamixel Torque
%write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, 1);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

% Write position (4 bytes)
% Resolution is 0-4095 ticks for 0-360degrees (e.g. 180 degrees = 2048
% ticks)
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, 0);
% pause(1);
% write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, 2048);

% i = 0;

timestep = 0.01
duration = 10
sineT = [0:timestep:duration];
sine    Pos = ( sin(sineT) + 1 ) / 2 * 4095 ;  % to ticks

tHist = []
tSetPointHist = []
tTime = []

for i = sinePos
%     [posHist, setPointHist, timeHist] = goToPos(port_num, DXL_ID, i);
%     [posHist, setPointHist, timeHist] = goToPos(port_num, DXL_ID, i);
%     tHist = [tHist, posHist];
%     tSetPointHist = [tSetPointHist, setPointHist];
%     if length(tTime) ~= 0
%         timeOffset = tTime(end);
%     else
%         timeOffset = 0;
%     end
%     tTime = [tTime, timeHist + timeOffset];
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, i);
end



% goToPos(0);
% goToPos(2048);
% [to0Hist, to0setPointHist, to0time] = goToPos(port_num, DXL_ID, 0);
% [to180Hist, to180setPointHist, to180time] = goToPos(port_num, DXL_ID, 2048);

%     j = 0;
%     set_point = 0;
%     dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_GOAL_POSITION, set_point);
% 
%     while (dxl_present_position ~= set_point)
%         
%         % Read present position
%         dxl_present_position = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_PRESENT_POSITION);
%         
%         % Error handling
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
%         
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
%         
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end
% 
%         fprintf('[ID:%03d] Position: %03d\n', DXL_ID, typecast(uint32(dxl_present_position), 'int32'));
% 
%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
%     end


% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);


% Plot history
% plot([to0time, to180time+to0time(end)], [to0Hist,to180Hist])
% hold on
% plot([to0time, to180time+to0time(end)], [to0setPointHist, to180setPointHist], "Color",'r')
% ylabel("Position / ticks")
% xlabel("Time / ms")

% Plot sine history
pos = plot(tTime, tHist)
hold on
setpoint = plot(tTime, tSetPointHist, "Color", 'r')
ylabel("Position / ticks")
xlabel("Time / ms")
legend([pos, setpoint], ["Position History", "Set point history"])


% close all;
clear all;