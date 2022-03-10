%% Code to test cubic interpolation

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

% ---- Control Table Addresses ---- %%

ADDR_PRO_TORQUE_ENABLE       = 64;           % Control table address is different in Dynamixel model
ADDR_PRO_GOAL_POSITION       = 116; 
ADDR_PRO_PRESENT_POSITION    = 132; 
ADDR_PRO_OPERATING_MODE      = 11;
ADDR_PRO_DRIVE_MODE          = 10;

ADDR_PRO_GOAL_VELOCITY       = 104;
ADDR_PRO_PROFILE_VELOCITY    = 112;



% ---- Other Settings ---- %%

% Protocol version
PROTOCOL_VERSION            = 2.0;          % See which protocol version is used in the Dynamixel

% Default setting
DXL_LIST = [11,12,13,14,15];
BAUDRATE                    = 1000000;
DEVICENAME                  = 'COM10';       % Check which port is being used on your controller
                                            % ex) Windows: 'COM1'   Linux: '/dev/ttyUSB0' Mac: '/dev/tty.usbserial-*'
                                            
TORQUE_ENABLE               = 1;            % Value for enabling the torque
TORQUE_DISABLE              = 0;            % Value for disabling the torque
DXL_MINIMUM_POSITION_VALUE  = -150000;      % Dynamixel will rotate between this value
DXL_MAXIMUM_POSITION_VALUE  = 150000;       % and this value (note that the Dynamixel would not move when the position value is out of movable range. Check e-manual about the range of the Dynamixel you use.)
DXL_MOVING_STATUS_THRESHOLD = 20;           % Dynamixel moving status threshold

ESC_CHARACTER               = 'e';          % Key for escaping loop

COMM_SUCCESS                = 0;            % Communication Success result value
COMM_TX_FAIL                = -1001;        % Communication Tx Failed

% ----- SET MOTION LIMITS ----------- %%
ADDR_MAX_POS = 48;
ADDR_MIN_POS = 52;
MAX_POS = 3400;
MIN_POS = 600;
% ---------------------------------- %%

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

%% Initialize all dynamixels
servoLimits = getServoLimits();

for i=1:length(DXL_LIST)
    % Put actuator into Position Control Mode
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_OPERATING_MODE, 3);
    % Set max position limit
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_MAX_POS, servoLimits(i,2));
    % Set min position limit
    write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_MIN_POS, servoLimits(i,1));
    % Set Dynamixel Torque
    write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE);

    % Set profile velocity - smoother motion
    % write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PROFILE_VELOCITY, 1024);
    % Profile acceleration

    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    else
        fprintf('Dynamixel %d has been successfully connected \n', i);
    end

end

%% Code snippet for testing
%% Code snippet for Line path sampling
z=90;
% Corners


square = [ 175 -50 z
           175  50 z
           75   50 z
           75  -50 z ];

square(5,:) = square(1,:);  % make it complete the square

% interpolate lines between corners
corners = [];
numPoints = 100;
for i=2:length(square)
    corners = [corners; linearInterpolate(square(i-1,:), square(i,:), numPoints) ];
end

GRIP_POS = deg2rad(232);
GRIP_ANGLE = 0;
        
%% ~ Code snippet for testing with standard inverse kinematics
% for j=1:length(corners)
%     theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
%     for i=1:length(DXL_LIST)
%         write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
%         % curr_servo_theta(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
%     end 
%     pause(5);
% end

%% ~ Code snippet for testing with waypoint interpolation
% Get thetas for corners[coeff]
vias = [];
for j=1:size(corners, 1)
    % corners(j,:)
    theta = inverseKinDynamixel(corners(j, 1), corners(j, 2), corners(j, 3), -pi/2, GRIP_POS);
    GRIP_ANGLE = theta(5);
    vias = [vias; theta(1:4)];
end
Tend = 30;

vias

% Interpolate between waypoints
[coeffs, T] = interpTraj(vias, Tend);

disp("TIME BASED");
start = now;

curr_servo_theta = zeros(1,5);
disp("Goal Theta");
goal_theta = [vias(end,:) GRIP_ANGLE]

% want to ensure we don't end up with a deadzone
% while(~all( abs(curr_servo_theta(1:4)-goal_theta(1:4)) < 50  ))
while 1
    curTime = (now - start) * 24*60*60; % Fractional part of `now` is 24hrs
                                        % we want to get current time in seconds
    if curTime<Tend
        theta = [sampleCubic(coeffs, T, curTime) GRIP_ANGLE];
    else
        % Clip to final theta position
        theta = [vias(end,:) GRIP_ANGLE];
        break
    end

    % fprintf("Commanded Pos at t:%0.2f | t_1: %0.1f | t_2: %0.1f | t_3: %0.1f | t_4: %0.1f | t_5: %0.1f\n", ...
    %     curTime, theta(1), theta(2), theta(3), theta(4), theta(5)    );
    % fprintf("Current Position         | t_1: %0.1f | t_2: %0.1f | t_3: %0.1f | t_4: %0.1f | t_5: %0.1f\n\n", ...
    %     curr_servo_theta(1), curr_servo_theta(2), curr_servo_theta(3), ...
    %     curr_servo_theta(4), curr_servo_theta(5));

    for i=1:length(DXL_LIST)
        write4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_GOAL_POSITION, theta(i));
        % curr_servo_theta(i) = read4ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_PRESENT_POSITION);
    end
end


%% -- Dynamixel Cleanup Start -- %%
for i=1:length(DXL_LIST)
    % Disable Dynamixel Torque
%     write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL_LIST(i), ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE);
    dxl_comm_result = getLastTxRxResult(port_num, PROTOCOL_VERSION);
    dxl_error = getLastRxPacketError(port_num, PROTOCOL_VERSION);
    if dxl_comm_result ~= COMM_SUCCESS
        fprintf('%s\n', getTxRxResult(PROTOCOL_VERSION, dxl_comm_result));
    elseif dxl_error ~= 0
        fprintf('%s\n', getRxPacketError(PROTOCOL_VERSION, dxl_error));
    end
end

% Close port
closePort(port_num);
fprintf('Port Closed \n');

% Unload Library
unloadlibrary(lib_name);

% close all;
clear all;