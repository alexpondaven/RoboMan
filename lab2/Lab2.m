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
DXL_ID_1                      = 14;            % Dynamixel ID: 1
DXL_ID_2                      = 15;            % Dynamixel ID: 1
DXL_LIST = [DXL_ID_1, DXL_ID_2];
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM8';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

%% ----- SET MOTION LIMITS ----------- %%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
%% ---------------------------------- %%

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
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_OPERATING_MODE, 3);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_OPERATING_MODE, 3);

% Set max position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MAX_POS, MAX_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MAX_POS, MAX_POS);
% Set min position limit
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_MIN_POS, MIN_POS);
write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_MIN_POS, MIN_POS);

% Set Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);

if dxl_comm_result ~= COMM_SUCCESS
    fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
elseif dxl_error ~= 0
    fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
else
    fprintf('Dynamixel has been successfully connected \n');
end

%% --- Dynamixel Setup End --- %%


% Code snippet for Task 1-1 (Read Encoders)
% i = 0;
% j = 0;
% while (j<1000)
%     j = j+1;
%     pos_arr = [0 0];
%     
%     for i = 1:length(DXL_LIST)
%         % Read present position
%         pos_arr(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
% 
%         % error handling
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
% 
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end
%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
%         %end error handling
%     end
%     
%     fprintf('[ID:%03d] Position: %03d\t', DXL_LIST(1), typecast(uint32(pos_arr(1)), 'int32'));
%     fprintf('[ID:%03d] Position: %03d\n', DXL_LIST(2), typecast(uint32(pos_arr(2)), 'int32'));
% end

% Code Snippet for Task 1-3 (Seq of pos commands) --> 90deg, 180deg, 270deg
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, 1024);
% end
% 
% pause(1);
% 
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, 2048);
% end
% 
% pause(1);
% 
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, 3072);
% end
% 
% pause(1);

% Code snippet for Task 1-4 (Three diff position commands)
% pos_arr = [1024, 3072];
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, pos_arr(i));
% end
% pause(1);
% 
% pos_arr = [3072, 1024];
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, pos_arr(i));
% end
% pause(1);
% 
% pos_arr = [3072, 2048];
% for i = 1:length(DXL_LIST)
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, pos_arr(i));
% end
% pause(1);

% Code snippet for Task 2
% timestep = 0.01;
% duration = 5;
% freq = 4;
% sineT = [0:timestep:duration];
% sinePos = ( sin(sineT*freq) * 500 ) + 2048;  % to ticks
% % amplitude of 500 ticks centered around 180deg (2048)
% 
% % Maths for inverting the sine wave for DXL_ID_2
% % rem_offset = i-2048
% % inv_offset = -(i-2048)
% % add_offset = -(i-2048) + 2048 = 4096 - i
% 
% for i = sinePos
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_GOAL_POSITION, i);
%     write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_GOAL_POSITION, 4096-i);
% %     Set the last value to i or 4096-i to give different servos different
% %     goal
% end

% Task 3-2 (2D Transformation Matrix) - we assume units are millimeters
T_0 = eye(3);   % World frame is always an identity matrix

% Task 3-3 frames
% T_1 = [
%         [cos(theta_1) -sin(theta_1)  0];
%         [sin(theta_1)  cos(theta_1)  0];
%         [0             0             1];
%       ];      % at the base of servo 1
%  
% T_2 = [
%         [1 0  0];
%         [0 1 80];
%         [0 0  1];
%       ];      % at the end of servo 1
% 
% T_3 = [
%         [cos(theta_2) -sin(theta_2)  0];
%         [sin(theta_2)  cos(theta_2)  0];
%         [0             0             1];
%       ];      % at the base of servo 2
%  
% T_4 = [
%         [1 0  0];
%         [0 1 60];
%         [0 0  1];
%       ];      % at the end of servo 2
  
% % Task 3-4 combine the matrices, plot histor
% i = 0;
% j = 0;
% servo_1_hist = zeros(1000,2);
% servo_2_hist = zeros(1000,2);
% while (j<500)
%     j = j+1;
%     pos_arr = [0 0];
%     
%     for i = 1:length(DXL_LIST)
%         % Read present position
%         pos_arr(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
% 
%         % error handling
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
% 
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end
%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
%         %end error handling
%     end
%     
%     % Convert positions into radians
%     pos_arr = ( pos_arr/4096*(2*pi) ) + pi; % 180deg on the servo is 0 radians
%     theta_1 = pos_arr(1);
%     theta_2 = pos_arr(2);
%     
%     % Get matrix multiplications
%     T_1 = [
%         [cos(theta_1) -sin(theta_1)  0];
%         [sin(theta_1)  cos(theta_1)  0];
%         [0             0             1];
%       ];      % at the base of servo 1
% 
%     T_2 = [
%             [1 0  0];
%             [0 1 80];
%             [0 0  1];
%           ];      % at the end of servo 1
% 
%     T_3 = [
%             [cos(theta_2) -sin(theta_2)  0];
%             [sin(theta_2)  cos(theta_2)  0];
%             [0             0             1];
%           ];      % at the base of servo 2
% 
%     T_4 = [
%             [1 0  0];
%             [0 1 60];
%             [0 0  1];
%           ];      % at the end of servo 2
%       
%     base_1 = (T_0 * T_1)* T_2;
%     base_2 = (base_1 * T_3) * T_4;  % make sure that order of ops is retained
%     
%     fprintf('[ID:%03d] X: %03f Y: %03f\t', DXL_LIST(1), base_1(1,3), base_1(2,3));
%     fprintf('[ID:%03d] X: %03f Y: %03f\n', DXL_LIST(2), base_2(1,3), base_2(2,3));
%     % extract the p_x and p_y elements of the base matrix
%     
%     % append to a history
%     servo_1_hist(j,:) = [base_1(1,3), base_1(2,3)];
%     servo_2_hist(j,:) = [base_2(1,3), base_2(2,3)];
% end
%  
% % Plot history
% servo1 = plot(servo_1_hist(:,1), servo_1_hist(:,2));
% hold on
% servo2 = plot(servo_2_hist(:,1), servo_2_hist(:,2));
% ylabel("ypos (mm)");
% xlabel("xpos (mm)");
% title("Team BOB");
% legend([servo1, servo2], ["servo1", "servo2"]);

% Task 4 Draw a Square
% i = 0;
% j = 0;
% servo_1_hist = zeros(1000,2);
% servo_2_hist = zeros(1000,2);
% while (j<1000)
%     j = j+1;
%     pos_arr = [0 0];
%     
%     for i = 1:length(DXL_LIST)
%         % Read present position
%         pos_arr(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%         dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
% 
%         % error handling
%         dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
% 
%         if dxl_comm_result ~= COMM_SUCCESS
%             fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
%         elseif dxl_error ~= 0
%             fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
%         end
%         if ~(abs(dxl_goal_position(index) - typecast(uint32(dxl_present_position), 'int32')) > DXL_MOVING_STATUS_THRESHOLD)
%             break;
%         end
%         %end error handling
%     end
%     
%     % Convert positions into radians
%     pos_arr_rad = ( pos_arr/4096*(2*pi) ) + pi; % 180deg on the servo is 0 radians
%     theta_1 = pos_arr_rad(1);
%     theta_2 = pos_arr_rad(2);
%     
%     % Get matrix multiplications
%     T_1 = [
%         [cos(theta_1) -sin(theta_1)  0];
%         [sin(theta_1)  cos(theta_1)  0];
%         [0             0             1];
%       ];      % at the base of servo 1
% 
%     T_2 = [
%             [1 0  0];
%             [0 1 80];
%             [0 0  1];
%           ];      % at the end of servo 1
% 
%     T_3 = [
%             [cos(theta_2) -sin(theta_2)  0];
%             [sin(theta_2)  cos(theta_2)  0];
%             [0             0             1];
%           ];      % at the base of servo 2
% 
%     T_4 = [
%             [1 0  0];
%             [0 1 60];
%             [0 0  1];
%           ];      % at the end of servo 2
%       
%     base_1 = (T_0 * T_1)* T_2;
%     base_2 = (base_1 * T_3) * T_4;  % make sure that order of ops is retained
%     
%     fprintf('[ID:%03d] Pos: %03d X: %03f Y: %03f\t', DXL_LIST(1), typecast(uint32(pos_arr(1)), 'int32'), base_1(1,3), base_1(2,3));
%     fprintf('[ID:%03d] Pos: %03d X: %03f Y: %03f\n', DXL_LIST(2), typecast(uint32(pos_arr(2)), 'int32'), base_2(1,3), base_2(2,3));
%     % extract the p_x and p_y elements of the base matrix
%     
%     % append to a history
%     servo_1_hist(j,:) = [base_1(1,3), base_1(2,3)];
%     servo_2_hist(j,:) = [base_2(1,3), base_2(2,3)];
% end
%  
% % Plot history
% servo1 = plot(servo_1_hist(:,1), servo_1_hist(:,2));
% hold on
% servo2 = plot(servo_2_hist(:,1), servo_2_hist(:,2));
% ylabel("ypos (mm)");
% xlabel("xpos (mm)");
% title("Team BOB");
% legend([servo1, servo2], ["servo1", "servo2"]);


% Moving in a square
% Clamp the bounds to the predefined MAX and MIN write values
servo1_pos_hist = [];
servo2_pos_hist = [];
servo_pos_hist = [servo1_pos_hist, servo2_pos_hist];

square_coords = [ [1809 600]; [1957 1511]; [2613 1420]; [2280 3400]; [1809 600]; ];
for i = 1:length(square_coords)
    for j = 1:length(DXL_LIST)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(j), ADDR_PRO_GOAL_POSITION, square_coords(i,j));
% Broken for now
%         poshist = [];
%         [poshist, ~, ~] = goToPos(port_num, DXL_LIST(j), square_coords(i,j));
%         servo_pos_hist(end+1, j) = pos_hist;    % append pos history
    end
    pause(2);
end

% Calculate transforms and plot x,y coords
% Get matrix multiplications
%     T_1 = [
%         [cos(theta_1) -sin(theta_1)  0];
%         [sin(theta_1)  cos(theta_1)  0];
%         [0             0             1];
%       ];      % at the base of servo 1
% 
%     T_2 = [
%             [1 0  0];
%             [0 1 80];
%             [0 0  1];
%           ];      % at the end of servo 1
% 
%     T_3 = [
%             [cos(theta_2) -sin(theta_2)  0];
%             [sin(theta_2)  cos(theta_2)  0];
%             [0             0             1];
%           ];      % at the base of servo 2
% 
%     T_4 = [
%             [1 0  0];
%             [0 1 60];
%             [0 0  1];
%           ];      % at the end of servo 2
%       
%     base_1 = (T_0 * T_1)* T_2;
%     base_2 = (base_1 * T_3) * T_4;  % make sure that order of ops is retained

%% -- Dynamixel Cleanup Start -- %%

% Disable Dynamixel Torque
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_1, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_ID_2, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
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

% close all;
clear all;